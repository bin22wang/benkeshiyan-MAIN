/**
 * @file wifi_mgr.h
 * @brief Wi-Fi STA 连接管理模块头文件
 *
 * 提供 Wi-Fi STA 连接的“最小封装”接口：创建 STA netif、配置 Wi-Fi、发起连接并阻塞等待获取 IP。
 *
 * 注意：本模块假设上层已完成全局网络初始化：
 *  - esp_netif_init()
 *  - esp_event_loop_create_default()
 * 否则 Wi-Fi/事件回调无法正常工作。
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Wi-Fi STA 并连接到预设 AP
 *
 * 内部会完成以下步骤：
 *  1. 创建默认 STA netif（esp_netif_create_default_wifi_sta）
 *  2. 初始化 Wi-Fi 驱动并注册事件回调（连接/断开/GOT_IP）
 *  3. 配置并启动 Wi-Fi STA 模式
 *  4. 阻塞等待连接成功或失败标志（最长 30 秒）
 *
 * Wi-Fi 的 SSID/密码等配置目前在实现文件中通过宏定义给出（wifi_mgr.c）。
 *
 * @return true  连接成功并获取到 IP
 * @return false 连接失败（超时或认证错误）
 */
bool wifi_mgr_init(void);

#ifdef __cplusplus
}
#endif
