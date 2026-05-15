/**
 * @file app_main.c
 * @brief 应用程序入口文件
 *
 * 负责协调各功能模块的初始化顺序，并注册业务回调：
 *  1. NVS 初始化（存储 Wi-Fi 配置等持久数据）
 *  2. LED 模块初始化（进入快速闪烁，指示未连接状态）
 *  3. Wi-Fi 模块初始化（阻塞至连接成功或超时）
 *  4. UDP 通信模块初始化（绑定端口，启动接收任务）
 *  5. 注册 UDP 接收回调（处理服务器下发的拓扑/控制/编队指令）
 *  6. 启动本车状态上报任务（10 Hz，格式 CAR_ID:x,y,yaw,vol,vx,vy,vz）
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "driver/gpio.h"

/* 各功能模块头文件 */
#include "modules/wifi_mgr.h"
#include "modules/udp_comm.h"
#include "modules/led_ctrl.h"
#include "modules/proto_parser.h"
#include "modules/recon_mgr.h"
#include "modules/uart.h"
#include "modules/vel_solver.h"
#include "modules/camera_ui.h"
#include "modules/aruco_pose.h"
#include "bsp_i2c.h"
#include "disp_mgr.h"
#include "xl9555.h"

/* ========== 模块日志标签 ========== */
static const char *TAG = "app_main";

/* Display role state (formation leader/follower overrides manual/position). */
static sys_mode_t s_role_mode = SYS_MODE_IDLE;

/* ========== UDP 通信配置 ========== */
/** 服务器 IP 地址 */
#define SERVER_IP       "192.168.31.46"
/** 服务器监听端口（ESP32 向此端口发送数据） */
#define SERVER_PORT     8080
/** 本地绑定端口（ESP32 在此端口接收服务器数据） */
#define LOCAL_PORT      8081
/** 本车 ID（由 MAC 地址映射得到） */
#define OWN_CAR_ID_DEFAULT "CAR1"

/* ========== MAC -> 车辆 ID 映射表 ========== */
/**
 * @brief MAC 地址到车辆 ID 的映射条目
 *
 * 用于在多车场景下，根据设备自身的 Wi-Fi STA MAC 推导出本车 ID。
 */
typedef struct {
    uint8_t mac[6];
    const char *car_id;
} mac_car_map_t;

static const mac_car_map_t k_mac_car_map[] = {
    {{0xDC, 0xB4, 0xD9, 0x16, 0xC1, 0x64}, "CAR1"},
    {{0xDC, 0xB4, 0xD9, 0x16, 0xD4, 0x58}, "CAR2"},
    {{0x3C, 0xDC, 0x75, 0x6C, 0x04, 0x80}, "CAR3"},
    {{0xDC, 0xB4, 0xD9, 0x16, 0xC7, 0x90}, "CAR4"},
};

static const char *lookup_car_id_by_mac(const uint8_t mac[6])
{
    for (size_t i = 0; i < sizeof(k_mac_car_map) / sizeof(k_mac_car_map[0]); i++) {
        if (memcmp(mac, k_mac_car_map[i].mac, sizeof(k_mac_car_map[i].mac)) == 0) {
            return k_mac_car_map[i].car_id;
        }
    }
    /* 未匹配到已知 MAC：回退到默认车辆 ID（避免空指针/未知 ID 影响业务逻辑） */
    return OWN_CAR_ID_DEFAULT;
}

/* ========== 状态上报间隔 ========== */
/** 向服务器上报本车状态的间隔（毫秒）；100 ms = 10 Hz，与 STM32 端发送频率一致 */
#define STATE_REPORT_INTERVAL_MS   100

/** 显示状态检查间隔（毫秒） */
#define DISP_UPDATE_INTERVAL_MS    50

/** BOOT 按键配置 */
#define BOOT_KEY_GPIO              GPIO_NUM_0
#define BOOT_KEY_ACTIVE_LEVEL      0
#define BOOT_KEY_DEBOUNCE_MS       30

/** LCD/XL9555 自检开关 */
#define LCD_DIAG_SELF_TEST         0

/* ========== 业务回调函数 ========== */

/**
 * @brief UDP 接收回调：处理服务器下发的指令
 *
 * 当 UDP 模块收到数据后，会调用此函数。
 * 目前支持解析两种消息格式：
 *  - "TOPOLOGY:..."        拓扑矩阵数据，供后续路径规划使用
 *  - "TOPOLOGY_TOGGLE:..." 拓扑切换指令
 *
 * @param data     接收到的字符串数据
 * @param len      数据长度（字节）
 * @param src_ip   来源 IP 地址
 * @param src_port 来源端口号
 */
static void on_udp_data_received(const char *data, size_t len,
                                 const char *src_ip, uint16_t src_port)
{
    /*
     * 注意：该回调运行在 UDP 接收任务（udp_rx_task）上下文。
     * 建议保持处理轻量；若需要做较重逻辑，可改为投递队列给其他任务处理。
     */
    ESP_LOGI(TAG, "[RX] from %s:%u, len=%d", src_ip, src_port, (int)len);

    /* 将原始 UDP 负载交给协议解析器（解析结果通过 consume/get 接口取出） */
    proto_parse_message(data, len);
    recon_mgr_handle_message(data, len);

    /* ------------------------------------------------------------------
     * 轮询解析结果并执行对应动作
     * ------------------------------------------------------------------ */

    /* --- MOVE 指令（通常是发给某个 car_id 的点位/航向目标） --- */
    proto_cmd_move_t mv;
    if (proto_consume_move_cmd(&mv)) {
        ESP_LOGI(TAG, "[CMD_MOVE] target=(%.3f, %.3f), yaw=%.1f",
                 mv.x, mv.y, mv.yaw);
        led_ctrl_set_mode(LED_MODE_ACTIVE);
        /*
         * vel_solver_set_position_target() 已在 proto_parse_message() 内部完成调用；
         * 速度输出由 uart_tx_task 每 20 ms 周期性下发给 STM32，此处无需额外操作。
         */
    }

    /* --- 编队/角色相关指令 --- */
    proto_cmd_formation_t fm;
    if (proto_consume_formation_cmd(&fm)) {
        /*
         * vel_solver 模式切换已在 proto_parse_message() 内部完成；
         * 此处只需根据角色更新 LED 状态，速度输出由 uart_tx_task 自动下发给 STM32。
         */
        switch (fm.type) {
            case FORMATION_CMD_STOP:
                ESP_LOGI(TAG, "[CMD_FORMATION] STOP");
                led_ctrl_set_mode(LED_MODE_CONNECTED);  /* 编队结束，恢复待命状态 */
                s_role_mode = SYS_MODE_IDLE;
                break;
            case FORMATION_CMD_SET_LEADER:
                ESP_LOGI(TAG, "[CMD_FORMATION] SET_LEADER");
                led_ctrl_set_mode(LED_MODE_ACTIVE);
                s_role_mode = SYS_MODE_LEADER;
                break;
            case FORMATION_CMD_SET_FOLLOWER:
                ESP_LOGI(TAG, "[CMD_FORMATION] FOLLOWER of %s, offset=(%.2f,%.2f,%.1f)",
                         fm.leader_id, fm.offset_x, fm.offset_y, fm.offset_yaw);
                led_ctrl_set_mode(LED_MODE_ACTIVE);
                s_role_mode = SYS_MODE_FOLLOW;
                break;
            case FORMATION_CMD_UPDATE_OFFSET:
                ESP_LOGI(TAG, "[CMD_FORMATION] UPDATE_OFFSET (%.2f,%.2f,%.1f)",
                         fm.offset_x, fm.offset_y, fm.offset_yaw);
                break;
            default:
                break;
        }
    }

    /* --- 拓扑信息：当前仅用于指示灯状态展示 --- */
    const proto_topology_t *topo = proto_get_topology();
    if (topo->enabled) {
        led_ctrl_set_mode(LED_MODE_ACTIVE);
    }
}

/* ========== 本车状态上报任务 ========== */

/**
 * @brief 状态上报任务：将 STM32 上报的本车位姿实时转发给服务器
 *
 * 上报频率：10 Hz（STATE_REPORT_INTERVAL_MS = 100 ms）
 * 上报格式（与 STM32 端 ESP8266_SendStatus_UDP_Reliable 保持一致）：
 *   "CAR_ID:x,y,yaw,volume,vx,vy,vz"
 * 其中 volume（电压）固定为 0.0（ESP32 无电压采集），
 * x/y 精度 %.2f，yaw 精度 %.1f，vx/vy/vz 精度 %.3f。
 *
 * 仅在 STM32 已通过 UART 上报有效状态后才发送，
 * 避免向服务器发送全零的无效数据。
 *
 * @param pvParameters  本车 ID 字符串（const char*），由 xTaskCreate 传入
 */
static void state_report_task(void *pvParameters)
{
    const char *own_car_id = (const char *)pvParameters;
    char msg[96];
    uint32_t fail_backoff_ms = STATE_REPORT_INTERVAL_MS;

    ESP_LOGI(TAG, "State report task running, interval=%d ms, car_id=%s",
             STATE_REPORT_INTERVAL_MS, own_car_id);

    while (1) {
        bool send_ok = false;
        const vel_own_state_t *st = vel_solver_get_own_state();
        if (st->valid) {
            /* 格式与 STM32 端 ESP8266_SendStatus_UDP_Reliable 保持一致：
             * CAR_ID:x,y,yaw,voltage,vx,vy,vz
             * voltage 固定 0.0（ESP32 无该采集） */
            int len = snprintf(msg, sizeof(msg),
                               "%s:%.2f,%.2f,%.1f,0.0,%.3f,%.3f,%.3f",
                               own_car_id,
                               st->x, st->y, st->yaw,
                               st->vx, st->vy, st->vz);
            send_ok = udp_comm_send(msg, (size_t)len);
            if (!send_ok) {
                ESP_LOGW(TAG, "State report send failed");
            }
        }

        if (send_ok) {
            fail_backoff_ms = STATE_REPORT_INTERVAL_MS;
        } else {
            fail_backoff_ms = (fail_backoff_ms < 1000U) ? (fail_backoff_ms * 2U) : 1000U;
        }

        vTaskDelay(pdMS_TO_TICKS(fail_backoff_ms));
    }

    vTaskDelete(NULL);
}

#if LCD_DIAG_SELF_TEST
static void lcd_diag_self_test(void)
{
    ESP_LOGI(TAG, "LCD diag: start");
    ESP_LOGI(TAG, "XL9555 I2C cfg: port=%d sda=%d scl=%d speed=%d addr=0x%02X",
             (int)XL9555_I2C_PORT,
             (int)XL9555_I2C_SDA_GPIO,
             (int)XL9555_I2C_SCL_GPIO,
             (int)XL9555_I2C_SPEED_HZ,
             (unsigned int)XL9555_ADDR);

    esp_err_t ret = xl9555_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "XL9555 init failed: %s", esp_err_to_name(ret));
        return;
    }

    uint8_t out_reg[2] = {0};
    if (xl9555_read_bytes(XL9555_OUTPUT_PORT0_REG, out_reg, sizeof(out_reg)) == ESP_OK) {
        uint16_t out_val = (uint16_t)((out_reg[1] << 8) | out_reg[0]);
        ESP_LOGI(TAG, "XL9555 OUT=0x%04X", out_val);
    } else {
        ESP_LOGW(TAG, "XL9555 read OUT failed");
    }

    uint8_t in_reg[2] = {0};
    if (xl9555_read_bytes(XL9555_INPUT_PORT0_REG, in_reg, sizeof(in_reg)) == ESP_OK) {
        uint16_t in_val = (uint16_t)((in_reg[1] << 8) | in_reg[0]);
        ESP_LOGI(TAG, "XL9555 IN=0x%04X", in_val);
    } else {
        ESP_LOGW(TAG, "XL9555 read IN failed");
    }

    ESP_LOGI(TAG, "LCD diag: toggle SLCD_PWR_IO");
    uint16_t val = xl9555_pin_write(SLCD_PWR_IO, 1);
    ESP_LOGI(TAG, "SLCD_PWR_IO=1 OUT=0x%04X", val);
    vTaskDelay(pdMS_TO_TICKS(80));
    val = xl9555_pin_write(SLCD_PWR_IO, 0);
    ESP_LOGI(TAG, "SLCD_PWR_IO=0 OUT=0x%04X", val);
    vTaskDelay(pdMS_TO_TICKS(80));
    val = xl9555_pin_write(SLCD_PWR_IO, 1);
    ESP_LOGI(TAG, "SLCD_PWR_IO=1 OUT=0x%04X", val);

    ESP_LOGI(TAG, "LCD diag: toggle SLCD_RST_IO");
    val = xl9555_pin_write(SLCD_RST_IO, 0);
    ESP_LOGI(TAG, "SLCD_RST_IO=0 OUT=0x%04X", val);
    vTaskDelay(pdMS_TO_TICKS(20));
    val = xl9555_pin_write(SLCD_RST_IO, 1);
    ESP_LOGI(TAG, "SLCD_RST_IO=1 OUT=0x%04X", val);

    ESP_LOGI(TAG, "LCD diag: done");
}
#endif

static sys_mode_t resolve_display_mode(vel_solver_mode_t mode)
{
    if (s_role_mode == SYS_MODE_LEADER || s_role_mode == SYS_MODE_FOLLOW) {
        return s_role_mode;
    }

    switch (mode) {
        case VEL_MODE_IDLE:
            return SYS_MODE_IDLE;
        case VEL_MODE_FOLLOWER:
            return SYS_MODE_FOLLOW;
        case VEL_MODE_MANUAL:
            return SYS_MODE_MANUAL;
        case VEL_MODE_POSITION:
        default:
            return SYS_MODE_MANUAL;
    }
}

static void display_task(void *pvParameters)
{
    const char *own_car_id = (const char *)pvParameters;
    uint32_t last_state_ts = 0;
    bool last_wifi_connected = false;
    bool last_pose_valid = false;
    float last_x = 0.0f;
    float last_y = 0.0f;
    float last_yaw = 0.0f;

    while (1) {
        const vel_own_state_t *own = vel_solver_get_own_state();
        bool wifi_connected = wifi_mgr_is_connected();

        if (own->valid && own->timestamp_ms != last_state_ts) {
            sys_mode_t mode = resolve_display_mode(vel_solver_get_mode());
            disp_mgr_update_status(own_car_id, wifi_connected, mode, own->x, own->y, own->yaw);
            last_state_ts = own->timestamp_ms;
            last_wifi_connected = wifi_connected;
            last_pose_valid = true;
            last_x = own->x;
            last_y = own->y;
            last_yaw = own->yaw;
        } else if (last_pose_valid && wifi_connected != last_wifi_connected) {
            sys_mode_t mode = resolve_display_mode(vel_solver_get_mode());
            disp_mgr_update_status(own_car_id, wifi_connected, mode, last_x, last_y, last_yaw);
            last_wifi_connected = wifi_connected;
        }
        vTaskDelay(pdMS_TO_TICKS(DISP_UPDATE_INTERVAL_MS));
    }
}

static void boot_key_task(void *pvParameters)
{
    (void)pvParameters;

    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOOT_KEY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    int last_level = gpio_get_level(BOOT_KEY_GPIO);
    while (1) {
        int level = gpio_get_level(BOOT_KEY_GPIO);
        if (level == BOOT_KEY_ACTIVE_LEVEL && last_level != BOOT_KEY_ACTIVE_LEVEL) {
            vTaskDelay(pdMS_TO_TICKS(BOOT_KEY_DEBOUNCE_MS));
            level = gpio_get_level(BOOT_KEY_GPIO);
            if (level == BOOT_KEY_ACTIVE_LEVEL) {
                while (gpio_get_level(BOOT_KEY_GPIO) == BOOT_KEY_ACTIVE_LEVEL) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                disp_mgr_next_page();
            }
        }
        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ========== 设备 MAC 打印 ========== */

/**
 * @brief 打印本机 Wi-Fi STA MAC 地址并返回映射后的车辆 ID
 */
static const char *print_own_mac(void)
{
    uint8_t mac[6] = {0};
    esp_err_t err = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read MAC failed: %s", esp_err_to_name(err));
        return OWN_CAR_ID_DEFAULT;
    }

    ESP_LOGI(TAG, "Wi-Fi STA MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return lookup_car_id_by_mac(mac);
}

/* ========== 应用入口 ========== */

/**
 * @brief 应用程序入口函数（ESP-IDF 框架调用）
 *
 * 按照以下顺序完成系统初始化：
 *  1. NVS 初始化
 *  2. LED 模块初始化（快速闪烁，表示启动中）
 *  3. Wi-Fi 连接（阻塞至成功或失败）
 *  4. UDP 通信初始化（创建 socket，启动接收任务）
 *  5. 注册 UDP 接收回调
 *  6. 启动状态上报任务（10 Hz 向服务器发送本车位姿）
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============ System Starting ============");

    if (bsp_i2c_master_init() != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed");
    }
    if (bsp_i2c_aux_init() != ESP_OK) {
        ESP_LOGW(TAG, "I2C1 init failed");
    }

    /* Reduce log spam so LCD diag logs are visible */
    esp_log_level_set("vel_solver", ESP_LOG_ERROR);

    /* --- Step 1: NVS init --- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase, re-initializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Step 1/5: NVS init OK");

    /* --- Global network init (call only once) --- */
    /* 初始化 TCP/IP 栈与默认事件循环：Wi-Fi、socket 等网络能力依赖它们 */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* --- Print device MAC --- */
    /* --- Print device MAC & resolve car id --- */
    const char *own_car_id = print_own_mac();
    ESP_LOGI(TAG, "Resolved own car id: %s", own_car_id);

    /* --- Protocol parser & vel solver init --- */
    proto_parser_init(own_car_id);
    vel_solver_init(); /* 清空解算器状态，必须在 proto_parser_init() 之后调用 */
    recon_mgr_init(own_car_id);

#if LCD_DIAG_SELF_TEST
    lcd_diag_self_test();
#endif
    disp_mgr_init();
    xTaskCreate(display_task, "display_task", 4096, (void *)own_car_id, 3, NULL);

    if (!camera_ui_init()) {
        ESP_LOGW(TAG, "Camera UI init failed");
    }

    aruco_pose_init();

    xTaskCreate(boot_key_task, "boot_key_task", 4096, NULL, 3, NULL);

    /* --- UART bridge init --- */
    /* 启动串口桥接任务：负责与 STM32 交换状态/控制量（详见 modules/uart.c） */
    uart_bridge_init(115200);
    uart_bridge_start();

    /* --- Step 2: LED init --- */
    led_ctrl_init();
    led_ctrl_set_mode(LED_MODE_DISCONNECTED);
    ESP_LOGI(TAG, "Step 2/5: LED init OK");

    /* --- Step 3: Wi-Fi connect --- */
    ESP_LOGI(TAG, "Step 3/5: Connecting to Wi-Fi...");
    if (!wifi_mgr_init()) {
        ESP_LOGE(TAG, "Wi-Fi connect failed, stop.");
        return;
    }
    led_ctrl_set_mode(LED_MODE_CONNECTED);
    ESP_LOGI(TAG, "Step 3/5: Wi-Fi connected");

    /* --- Step 4: UDP init --- */
    ESP_LOGI(TAG, "Step 4/5: UDP init...");
    if (!udp_comm_init(SERVER_IP, SERVER_PORT, LOCAL_PORT)) {
        ESP_LOGE(TAG, "UDP init failed, stop.");
        return;
    }
    ESP_LOGI(TAG, "Step 4/5: UDP init OK");
    /* --- Step 5: Register RX callback --- */
    udp_comm_register_rx_callback(on_udp_data_received);
    ESP_LOGI(TAG, "Step 5/5: UDP RX callback registered");

    /* --- Start state report task --- */
    /* 将 own_car_id 字符串作为任务参数传入；指针指向 k_mac_car_map 中的字符串字面量，
     * 生命周期覆盖整个程序运行，无需额外内存管理 */
    xTaskCreate(state_report_task, "state_report", 4096, (void *)own_car_id, 4, NULL);
    ESP_LOGI(TAG, "State report task started, interval=%d ms, car_id=%s",
             STATE_REPORT_INTERVAL_MS, own_car_id);

    ESP_LOGI(TAG, "============ System Ready, server=%s:%d, local_port=%d ============",
             SERVER_IP, SERVER_PORT, LOCAL_PORT);

    /*
     * app_main 返回后 ESP-IDF 会自动删除该任务，
     * 系统将由 FreeRTOS 调度其他任务（udp_rx_task、heartbeat_task、led_blink_task）继续运行。
     */
}
