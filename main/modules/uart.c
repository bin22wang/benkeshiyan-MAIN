/**
 * @file uart.c
 * @brief ESP32 <-> STM32 串口桥接（UART2，引脚见 uart.h 宏定义）
 *
 * 串口通信约定（当前实现）：
 *  - 物理层：UART（见 uart.h 中 USART_UX/引脚定义）
 *  - 帧边界：以换行符 '\n' 作为一帧结束（兼容 "\r\n"）
 *  - RX（STM32 -> ESP32）："[x,y,heading,vx,vy,vz]\r\n"（含方括号，6 个 %.3f）
 *  - TX（ESP32 -> STM32）："[vx,vy,vz]\r\n"（3 个 %.4f，逗号分隔）
 *
 * 数据流：
 *  - RX 任务：uart_read_bytes() -> 按字节拼接一行 -> sscanf 解析 -> vel_solver_update_own_state()
 *  - TX 任务：周期计算 vel_solver_compute() -> uart_bridge_send_target() 发送目标速度
 */

#include "uart.h"

#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "vel_solver.h"
#include <math.h>

static const char *TAG = "uart_bridge";

/* 发送任务周期（ms） */
#define UART_TX_INTERVAL_MS 20

/* -----------------------------------------------------------------------
 * TX 智能发送参数
 *
 * 优化策略：
 *  1. 活跃模式：仅当速度较上一发送帧变化超过死区时才发送；
 *     接近目标稳定时每 KEEPALIVE_FRAMES 帧补发一次，防止 STM32 超时清零。
 *  2. IDLE 模式：进入时立刻发一帧零速令小车停止；
 *     之后仅每 IDLE_FRAMES 帧补一次心跳，大幅减少无效 UART 流量。
 * ----------------------------------------------------------------------- */

/** 速度变化死区（m/s 或 rad·s??）：低于此值视为「无变化」，跳过本帧发送 */
#define UART_TX_DEADBAND          0.001f

/** 活跃模式稳定时的 keepalive 间隔（帧数）；5 × 20 ms = 100 ms */
#define UART_TX_KEEPALIVE_FRAMES  5

/** IDLE 模式零速 keepalive 间隔（帧数）；50 × 20 ms = 1 s */
#define UART_TX_IDLE_FRAMES       50

/* 解析缓冲 */
static char s_line_buf[RX_BUF_SIZE] = {0};
static size_t s_line_len = 0;

static void uart_rx_task(void *pvParameters);
static void uart_tx_task(void *pvParameters);

void uart_bridge_init(uint32_t baudrate)
{
    /* UART 基础参数（8N1，无硬件流控） */
    uart_config_t uart_config = {0};
    uart_config.baud_rate = (int)baudrate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    /* 应用 UART 参数到指定端口 */
    ESP_ERROR_CHECK(uart_param_config(USART_UX, &uart_config));

    /* 设置 TX/RX 引脚（RTS/CTS 不使用，用 UART_PIN_NO_CHANGE） */
    ESP_ERROR_CHECK(uart_set_pin(USART_UX,
                                 USART_TX_GPIO_PIN,
                                 USART_RX_GPIO_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    /*
     * 安装 UART driver：
     *  - RX/TX ring buffer：各 2*RX_BUF_SIZE
     *  - 事件队列：此处不使用（queue=NULL），改为轮询 uart_read_bytes()
     */
    ESP_ERROR_CHECK(uart_driver_install(USART_UX,
                                        RX_BUF_SIZE * 2,
                                        RX_BUF_SIZE * 2,
                                        20,
                                        NULL,
                                        0));

    ESP_LOGI(TAG, "UART init OK (port=%d, tx=%d, rx=%d, baud=%u)",
             (int)USART_UX,
             (int)USART_TX_GPIO_PIN,
             (int)USART_RX_GPIO_PIN,
             (unsigned int)baudrate);
}

void uart_bridge_start(void)
{
    /* 创建收发任务：
     *  - RX 优先级略高，尽量及时取走串口缓冲，避免溢出
     *  - TX 定时发送目标速度
     */
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 6, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", 4096, NULL, 5, NULL);
}

bool uart_bridge_send_target(float target_vx, float target_vy, float target_vz)
{
    char msg[96];
    /* 发送 3 个目标速度，ASCII CSV，一行一帧，以 '\n' 结束 */
    int len = snprintf(msg, sizeof(msg), "[%.4f,%.4f,%.4f]\r\n",
                       target_vx, target_vy, target_vz);
    if (len <= 0) return false;
    /* uart_write_bytes(): 将数据写入 UART driver 的 TX ring buffer，返回实际写入字节数 */
    int written = uart_write_bytes(USART_UX, msg, len);
    return written == len;
}

static void handle_state_line(const char *line)
{
    /* STM32 上报格式（含方括号）：[x,y,heading,vx,vy,vz] */
    float x = 0.0f, y = 0.0f, heading = 0.0f, vx = 0.0f, vy = 0.0f, vz = 0.0f;
    int n = sscanf(line, "[%f,%f,%f,%f,%f,%f]", &x, &y, &heading, &vx, &vy, &vz);
    if (n != 6) {
        ESP_LOGW(TAG, "RX parse failed (got %d fields): %s", n, line);
        return;
    }

    vel_solver_update_own_state(x, y, heading, vx, vy, vz);

    /* 每 50 帧（约 5 s @10 Hz）打印一次，避免高频日志引入调度抖动 */
    static uint32_t s_rx_log_cnt = 0;
    if ((s_rx_log_cnt++ % 50) == 0) {
        ESP_LOGI(TAG, "RX state: x=%.3f y=%.3f yaw=%.1f vx=%.3f vy=%.3f vz=%.3f",
                 x, y, heading, vx, vy, vz);
    }
}

static void uart_rx_task(void *pvParameters)
{
    uint8_t rxbuf[128];
    while (1) {
        /*
         * 轮询读取串口数据：
         *  - timeout=50ms：避免永久阻塞，便于任务被调度/看门狗观测
         *  - len>0：表示本次取到的字节数
         */
        int len = uart_read_bytes(USART_UX, rxbuf, sizeof(rxbuf), pdMS_TO_TICKS(50));
        if (len > 0) {
            /* 按字节做“行”拼接：遇到 '\n' 认为一帧结束，交给 handle_state_line() */
            for (int i = 0; i < len; i++) {
                char c = (char)rxbuf[i];
                /* 兼容 CRLF：忽略 '\r' */
                if (c == '\r') continue;
                if (c == '\n') {
                    /* 行结束：补 '\0' 形成 C 字符串 */
                    s_line_buf[s_line_len] = '\0';
                    if (s_line_len > 0) {
                        /* 解析并应答（ACK/ERR 在 handle_state_line 内发送） */
                        handle_state_line(s_line_buf);
                    }
                    /* 重置行缓冲，开始接收下一行 */
                    s_line_len = 0;
                    continue;
                }
                if (s_line_len < sizeof(s_line_buf) - 1) {
                    s_line_buf[s_line_len++] = c;
                } else {
                    /* 行过长（可能对端协议异常/丢字节）：丢弃当前行，避免越界 */
                    s_line_len = 0;
                }
            }
        }
    }
}

static void uart_tx_task(void *pvParameters)
{
    uint32_t tick          = 0;
    uint32_t frames_no_send = 0;  /* 距上次实际发送已过去的帧数 */
    float    last_vx = 0.0f, last_vy = 0.0f, last_vz = 0.0f;
    bool     last_valid = false;

    while (1) {
        vel_output_t out = {0};
        bool  valid = vel_solver_compute(&out) && out.valid;
        float tx_vx = valid ? out.target_vx : 0.0f;
        float tx_vy = valid ? out.target_vy : 0.0f;
        float tx_vz = valid ? out.target_vz : 0.0f;

        bool should_send = false;

        if (!valid) {
            /*
             * IDLE 模式：
             *  - 刚从活跃切换进 IDLE → 立即发一帧零速，令 STM32 立刻停车
             *  - 之后每 UART_TX_IDLE_FRAMES（1 s）补一次心跳
             *    （若 STM32 有串口超时保护，此心跳可防止误触发）
             */
            should_send = last_valid || (frames_no_send >= UART_TX_IDLE_FRAMES);
        } else {
            /*
             * 活跃控制模式：
             *  - 刚从 IDLE 切换进来 → 立即发送
             *  - 速度较上帧变化 > UART_TX_DEADBAND → 立即发送
             *  - 速度稳定（死区内，如已到达目标点）→ 每 100 ms 补一帧 keepalive，
             *    防止 STM32 因超时将速度清零，同时避免对稳定电机反复写相同值
             */
            bool mode_changed = !last_valid;
            bool vel_changed  = fabsf(tx_vx - last_vx) > UART_TX_DEADBAND ||
                                fabsf(tx_vy - last_vy) > UART_TX_DEADBAND ||
                                fabsf(tx_vz - last_vz) > UART_TX_DEADBAND;
            bool keepalive    = (frames_no_send >= UART_TX_KEEPALIVE_FRAMES);
            should_send = mode_changed || vel_changed || keepalive;
        }

        if (should_send) {
            uart_bridge_send_target(tx_vx, tx_vy, tx_vz);
            last_vx = tx_vx;  last_vy = tx_vy;  last_vz = tx_vz;
            frames_no_send = 0;

            if ((tick % 50) == 0) {
                if (valid) {
                    ESP_LOGI(TAG, "TX target: vx=%.3f vy=%.3f vz=%.3f",
                             tx_vx, tx_vy, tx_vz);
                } else {
                    ESP_LOGD(TAG, "TX idle keepalive");
                }
            }
        } else {
            frames_no_send++;
        }

        last_valid = valid;
        tick++;

        /* 固定节拍（20 ms）：任务唤醒频率不变，只是有时跳过发送 */
        vTaskDelay(pdMS_TO_TICKS(UART_TX_INTERVAL_MS));
    }
}
