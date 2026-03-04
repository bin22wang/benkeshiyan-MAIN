/**
 * @file wifi_mgr.h
 * @brief Wi-Fi STA 连接管理模块头文件
 *
 * 提供 Wi-Fi 初始化、连接、等待 IP 获取等接口。
 * 上层代码只需调用 wifi_mgr_init() 即可完成 Wi-Fi STA 连接。
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
 *  1. 初始化 esp_netif 和事件循环
 *  2. 配置并启动 Wi-Fi STA 模式
 *  3. 阻塞等待连接成功或超时（最长 30 秒）
 *
 * @return true  连接成功并获取到 IP
 * @return false 连接失败（超时或认证错误）
 */
bool wifi_mgr_init(void);

#ifdef __cplusplus
}
#endif
