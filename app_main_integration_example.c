/**
 * @file app_main_integration_example.c
 * @brief app_main.c 中集成 video_streamer 模块的 COMPLETE 代码示例
 *
 * 本文件展示了如何在现有 app_main.c 中集成视频流传输功能。
 * 你可以将相关代码复制到实际的 app_main.c 中。
 *
 * ?? 重要说明：这是示例代码，请根据实际的 app_main.c 结构进行调整。
 */

// ============================================================================
// 第 1 步：在 app_main.c 顶部添加以下头文件包含
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_camera.h"

// ===== 项目模块 =====
#include "disp_mgr.h"
#include "wifi_mgr.h"
#include "udp_comm.h"
#include "proto_parser.h"
#include "camera_ui.h"
#include "recon_mgr.h"
#include "led_ctrl.h"

// ===== 新增：视频流模块 =====
#include "video_streamer.h"


static const char *TAG = "app_main";


// ============================================================================
// 第 2 步：添加 UDP 数据接收回调函数（如果还没有）
// ============================================================================

/**
 * @brief UDP 接收回调函数
 *
 * 该函数在 UDP 接收任务中被调用，处理所有收到的 UDP 数据。
 * 其中包括 V 协议（视频流控制）的处理。
 */
static void on_udp_data_received(const char *data, size_t len,
                                 const char *src_ip, uint16_t src_port)
{
    /*
     * 注意：该回调运行在 UDP 接收任务（udp_rx_task）上下文。
     * 建议保持处理轻量；若需要做较重逻辑，可改为投递队列给其他任务处理。
     */
    ESP_LOGI(TAG, "[RX] from %s:%u, len=%d", src_ip, src_port, (int)len);

    // 将原始 UDP 负载交给协议解析器（解析结果通过 consume/get 接口取出）
    proto_parse_message(data, len);
    recon_mgr_handle_message(data, len);

    // ========================================================================
    // ===== 新增：处理 V 协议视频流控制命令 =====
    // ========================================================================
    proto_video_cmd_t video_cmd = proto_consume_video_cmd();
    switch (video_cmd) {
        case VIDEO_CMD_START:
            ESP_LOGI(TAG, "? Video streaming START command received");
            video_streamer_start();
            // 可选：发送确认消息回服务器
            // udp_comm_send("[V,ACK,START]", 13);
            break;

        case VIDEO_CMD_STOP:
            ESP_LOGI(TAG, "? Video streaming STOP command received");
            video_streamer_stop();
            // 可选：发送确认消息回服务器
            // udp_comm_send("[V,ACK,STOP]", 12);
            break;

        case VIDEO_CMD_NONE:
            /* 无新的视频命令，继续处理其他逻辑 */
            break;
    }
    // ========================================================================
}


// ============================================================================
// 第 3 步：修改 app_main 函数，添加 video_streamer_init() 调用
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Starting main application...");

    /* ========== 存储初始化 ========== */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "? NVS initialized");

    /* ========== 显示屏初始化 ========== */
    ESP_ERROR_CHECK(disp_mgr_init());
    ESP_LOGI(TAG, "? Display manager initialized");

    /* ========== 摄像头初始化 ========== */
    if (!camera_ui_init()) {
        ESP_LOGE(TAG, "? Camera init failed!");
        return;
    }
    ESP_LOGI(TAG, "? Camera UI initialized");

    /* ========== Wi-Fi 初始化 ========== */
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_mgr_init();
    ESP_LOGI(TAG, "? Wi-Fi manager initialized");

    /* ========== 等待 Wi-Fi 连接 ========== */
    // （根据实际需求等待 Wi-Fi 连接...）
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* ========== UDP 通信初始化 ========== */
    // 注意：这里的 IP 和端口需要根据实际网络配置调整
    const char *server_ip = "192.168.31.46";      // 服务器 IP
    uint16_t server_port = 5000;                  // 服务器端口（发送目标）
    uint16_t local_port = 5001;                   // 本地接收端口

    if (!udp_comm_init(server_ip, server_port, local_port)) {
        ESP_LOGE(TAG, "? UDP communication init failed!");
        return;
    }
    ESP_LOGI(TAG, "? UDP communication module initialized");

    /* ========== 注册 UDP 接收回调 ========== */
    udp_comm_register_rx_callback(on_udp_data_received);
    ESP_LOGI(TAG, "? UDP RX callback registered");

    /* ========== 协议解析器初始化 ========== */
    proto_parser_init("CAR1");  // 本车 ID
    ESP_LOGI(TAG, "? Protocol parser initialized");

    /* ========== 其他模块初始化 ========== */
    recon_mgr_init();
    ESP_LOGI(TAG, "? Reconnaissance manager initialized");

    led_ctrl_init();
    ESP_LOGI(TAG, "? LED controller initialized");

    /* ========================================================================
     * ===== 新增：视频流监控模块初始化 =====
     * 
     * 必须在以下模块之后调用：
     *   - camera_ui_init()      （摄像头初始化完成）
     *   - udp_comm_init()       （UDP 通信就绪）
     * ======================================================================*/
    video_streamer_init();
    ESP_LOGI(TAG, "? Video streamer module initialized");
    /* ======================================================================*/

    ESP_LOGI(TAG, "? All systems initialized successfully!");

    /* ========== 主应用循环 ========== */
    // 根据实际工程的应用循环继续...
    while (1) {
        // 例如：处理其他后台任务、状态监测等
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 可选：定期打印系统状态
        // ESP_LOGI(TAG, "System tick: %ld", xTaskGetTickCount());
    }
}


// ============================================================================
// 第 4 步（可选）：添加远程互动任务
// ============================================================================

/**
 * @brief 远程交互任务示例
 *
 * 该任务定期处理各种协议消息并执行相应的控制命令。
 * 这是一个可选的示例，展示如何在主应用流中集成视频流控制。
 */
static void remote_control_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Remote control task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));  // 每 100ms 检查一次

        // 消费移动指令
        proto_cmd_move_t move_cmd;
        if (proto_consume_move_cmd(&move_cmd)) {
            ESP_LOGI(TAG, "Move command: target=%s, pos=(%.2f, %.2f, %.2f)",
                     move_cmd.target_car, move_cmd.x, move_cmd.y, move_cmd.yaw);
            // 调用运动控制函数...
        }

        // 消费编队指令
        proto_cmd_formation_t form_cmd;
        if (proto_consume_formation_cmd(&form_cmd)) {
            ESP_LOGI(TAG, "Formation command: type=%d, leader=%s",
                     form_cmd.type, form_cmd.leader_id);
            // 调用编队控制函数...
        }

        // 其他逻辑...
    }

    vTaskDelete(NULL);
}


// ============================================================================
// 附加说明
// ============================================================================

/*
 * ? 集成检查清单：
 *
 * 1. [ ] 在 CMakeLists.txt 中添加了 "modules/video_streamer.c"
 * 2. [ ] 在 app_main.c 顶部添加了 #include "video_streamer.h"
 * 3. [ ] 在 app_main() 中调用了 video_streamer_init()
 * 4. [ ] 在 UDP 回调中添加了 video_cmd 处理代码
 * 5. [ ] 注册了 UDP 接收回调 (udp_comm_register_rx_callback)
 * 6. [ ] 确保摄像头、UDP、proto_parser 都已在 video_streamer_init 前初始化
 *
 * ?? 常见问题：
 *
 * Q: 编译时报 "undefined reference to video_streamer_init"
 * A: 检查 CMakeLists.txt 是否包含 video_streamer.c，执行 idf.py clean
 *
 * Q: 启动后没有图传数据发送
 * A: 确认 camera_ui_init() 和 udp_comm_init() 都成功了
 *    查看日志中是否有 video_streamer 的 ERROR
 *
 * Q: 收到 [V,START] 后小车没有反应
 * A: 确保 proto_consume_video_cmd() 在 UDP 回调中被调用
 *    检查 app_main 中的集成代码是否正确复制
 */
