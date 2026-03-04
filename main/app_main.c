/**
 * @file app_main.c
 * @brief 应用程序入口文件
 *
 * 负责协调各功能模块的初始化顺序，并注册业务回调：
 *  1. NVS 初始化（存储 Wi-Fi 配置等持久数据）
 *  2. LED 模块初始化（进入快速闪烁，指示未连接状态）
 *  3. Wi-Fi 模块初始化（阻塞至连接成功或超时）
 *  4. UDP 通信模块初始化（绑定端口，启动接收任务）
 *  5. 注册 UDP 接收回调（处理服务器下发的拓扑/控制指令）
 *  6. 启动周期性心跳发送任务
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"

/* 各功能模块头文件 */
#include "modules/wifi_mgr.h"
#include "modules/udp_comm.h"
#include "modules/led_ctrl.h"
#include "modules/proto_parser.h"

/* ========== 模块日志标签 ========== */
static const char *TAG = "app_main";

/* ========== UDP 通信配置 ========== */
/** 服务器 IP 地址 */
#define SERVER_IP       "192.168.31.46"
/** 服务器监听端口（ESP32 向此端口发送数据） */
#define SERVER_PORT     8080
/** 本地绑定端口（ESP32 在此端口接收服务器数据） */
#define LOCAL_PORT      8081
/** 本车 ID（与 STM32 MAC 地址绑定，此处先写固定值，后续可从 NVS 读取） */
#define OWN_CAR_ID      "CAR1"

/* ========== 心跳发送间隔 ========== */
/** 向服务器发送心跳包的间隔（毫秒） */
#define HEARTBEAT_INTERVAL_MS   1000

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
    ESP_LOGI(TAG, "[RX] from %s:%u, len=%d", src_ip, src_port, (int)len);

    /* Hand raw payload to the protocol parser */
    proto_parse_message(data, len);

    /* ------------------------------------------------------------------ *
     * Poll parsed results and act on them                                 *
     * ------------------------------------------------------------------ */

    /* --- Move command (addressed to OWN_CAR_ID) --- */
    proto_cmd_move_t mv;
    if (proto_consume_move_cmd(&mv)) {
        ESP_LOGI(TAG, "[CMD_MOVE] target=(%.3f, %.3f), yaw=%.1f",
                 mv.x, mv.y, mv.yaw);
        led_ctrl_set_mode(LED_MODE_ACTIVE);
        /* TODO: forward to STM32 via UART */
    }

    /* --- Formation command --- */
    proto_cmd_formation_t fm;
    if (proto_consume_formation_cmd(&fm)) {
        switch (fm.type) {
            case FORMATION_CMD_STOP:
                ESP_LOGI(TAG, "[CMD_FORMATION] STOP");
                break;
            case FORMATION_CMD_SET_LEADER:
                ESP_LOGI(TAG, "[CMD_FORMATION] SET_LEADER");
                break;
            case FORMATION_CMD_SET_FOLLOWER:
                ESP_LOGI(TAG, "[CMD_FORMATION] FOLLOWER of %s, offset=(%.2f,%.2f,%.1f)",
                         fm.leader_id, fm.offset_x, fm.offset_y, fm.offset_yaw);
                break;
            case FORMATION_CMD_UPDATE_OFFSET:
                ESP_LOGI(TAG, "[CMD_FORMATION] UPDATE_OFFSET (%.2f,%.2f,%.1f)",
                         fm.offset_x, fm.offset_y, fm.offset_yaw);
                break;
            default:
                break;
        }
        /* TODO: forward to STM32 via UART */
    }

    /* --- Topology update → just log for now --- */
    const proto_topology_t *topo = proto_get_topology();
    if (topo->enabled) {
        led_ctrl_set_mode(LED_MODE_ACTIVE);
    }
}

/* ========== 周期性心跳任务 ========== */

/**
 * @brief 心跳发送任务
 *
 * 每隔 HEARTBEAT_INTERVAL_MS 毫秒向服务器发送一条心跳消息，
 * 格式为："HELLO:esp32s3,seq=<序号>"
 * 服务器可凭此感知 ESP32 的在线状态。
 *
 * @param pvParameters 未使用
 */
static void heartbeat_task(void *pvParameters)
{
    int seq = 0;
    char msg[64];

    ESP_LOGI(TAG, "Heartbeat task running, interval=%d ms", HEARTBEAT_INTERVAL_MS);

    while (1) {
        int len = snprintf(msg, sizeof(msg), "HELLO:esp32s3,seq=%d", seq++);
        if (!udp_comm_send(msg, (size_t)len)) {
            ESP_LOGW(TAG, "Heartbeat send failed, seq=%d", seq - 1);
        }
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }

    vTaskDelete(NULL);
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
 *  6. 启动心跳发送任务
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============ System Starting ============");

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
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* --- Protocol parser init --- */
    proto_parser_init(OWN_CAR_ID);

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

    /* --- Start heartbeat task --- */
    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "Heartbeat task started, interval=%d ms", HEARTBEAT_INTERVAL_MS);

    ESP_LOGI(TAG, "============ System Ready, server=%s:%d, local_port=%d ============",
             SERVER_IP, SERVER_PORT, LOCAL_PORT);

    /*
     * app_main 返回后 ESP-IDF 会自动删除该任务，
     * 系统将由 FreeRTOS 调度其他任务（udp_rx_task、heartbeat_task、led_blink_task）继续运行。
     */
}
