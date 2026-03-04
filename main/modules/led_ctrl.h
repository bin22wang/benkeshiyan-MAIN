/**
 * @file led_ctrl.h
 * @brief LED 状态指示控制模块头文件
 *
 * 通过 GPIO 控制板载 LED，以不同的闪烁模式反映系统运行状态：
 *  - 未连接 Wi-Fi：快速闪烁（500ms 间隔）
 *  - Wi-Fi 已连接，等待 UDP：慢速闪烁（1000ms 间隔）
 *  - UDP 通信活跃：常亮
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LED 工作模式枚举
 */
typedef enum {
    LED_MODE_DISCONNECTED = 0,  /*!< Wi-Fi 未连接：快速闪烁（500ms） */
    LED_MODE_CONNECTED,         /*!< Wi-Fi 已连接，UDP 空闲：慢速闪烁（1000ms） */
    LED_MODE_ACTIVE,            /*!< UDP 正在收发数据：常亮 */
} led_mode_t;

/**
 * @brief 初始化 LED 控制模块
 *
 * 配置 LED 对应的 GPIO 为输出模式，
 * 并启动后台 FreeRTOS 闪烁控制任务。
 * 初始状态为 LED_MODE_DISCONNECTED（快速闪烁）。
 */
void led_ctrl_init(void);

/**
 * @brief 设置 LED 工作模式
 *
 * 可在任意任务中调用，线程安全（仅写一个 volatile 变量）。
 *
 * @param mode 目标模式，参见 led_mode_t 枚举
 */
void led_ctrl_set_mode(led_mode_t mode);

#ifdef __cplusplus
}
#endif
