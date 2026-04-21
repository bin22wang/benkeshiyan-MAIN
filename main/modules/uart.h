/**
 * @file uart.h
 * @brief ESP32 <-> STM32 串口桥接模块（UART2，引脚见下方宏定义）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 引脚和串口定义 */
/*
 * 本模块当前固定使用 UART2。
 * 如需改为其他 UART 口或引脚，仅需修改以下宏即可（并确保硬件连线一致）。
 */
#define USART_UX            UART_NUM_2
#define USART_TX_GPIO_PIN   GPIO_NUM_46
#define USART_RX_GPIO_PIN   GPIO_NUM_3

/* 串口接收相关定义 */
/*
 * uart_driver_install() 的 ring buffer 大小（字节）。
 * 注意：这不是“单行协议”的最大长度；单行解析缓冲见 uart.c 中的 s_line_buf。
 */
#define RX_BUF_SIZE         1024

/**
 * @brief 初始化串口桥接
 * @param baudrate 波特率
 */
void uart_bridge_init(uint32_t baudrate);

/**
 * @brief 启动串口收发任务
 */
void uart_bridge_start(void);

/**
 * @brief 发送目标速度到 STM32（target_Vx,target_Vy,target_Vz）
 *
 * 当前发送格式为 ASCII CSV，一行一帧，以 '\n' 结束：
 *   "<vx>,<vy>,<vz>\n"
 * 其中每个值保留 4 位小数。
 *
 * @note 该函数仅负责写入 UART 发送缓冲；对端协议解析与时序保证由上层约定。
 * @return true 发送成功
 */
bool uart_bridge_send_target(float target_vx, float target_vy, float target_vz);

#ifdef __cplusplus
}
#endif
