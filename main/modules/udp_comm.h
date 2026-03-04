/**
 * @file udp_comm.h
 * @brief UDP 通信模块头文件
 *
 * 提供 UDP socket 的初始化、数据发送、接收任务启动及回调注册接口。
 * 上层只需注册一个回调函数，即可在收到 UDP 数据时被通知。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UDP 接收回调函数类型
 *
 * 当接收到 UDP 数据包时，模块内部将调用此回调。
 *
 * @param data   接收到的数据缓冲区指针（已添加字符串终止符）
 * @param len    数据长度（字节数，不含终止符）
 * @param src_ip 数据包来源 IP 地址字符串（如 "192.168.31.46"）
 * @param src_port 数据包来源端口号
 */
typedef void (*udp_rx_callback_t)(const char *data, size_t len,
                                  const char *src_ip, uint16_t src_port);

/**
 * @brief 初始化 UDP 通信模块
 *
 * 完成以下操作：
 *  1. 创建 UDP socket
 *  2. 绑定本地接收端口
 *  3. 设置接收超时，防止 recvfrom 永久阻塞
 *  4. 启动后台接收任务
 *
 * @param server_ip   服务器 IP 地址字符串（如 "192.168.31.46"）
 * @param server_port 服务器端口号（发送目标端口）
 * @param local_port  本地绑定端口号（用于接收数据）
 * @return true  初始化成功
 * @return false 初始化失败（socket/bind 错误）
 */
bool udp_comm_init(const char *server_ip, uint16_t server_port,
                   uint16_t local_port);

/**
 * @brief 向服务器发送 UDP 数据
 *
 * 将数据通过已初始化的 socket 发送到服务器。
 * 必须在 udp_comm_init() 成功后调用。
 *
 * @param data 要发送的数据指针
 * @param len  数据长度（字节数）
 * @return true  发送成功
 * @return false 发送失败（socket 未初始化或 sendto 错误）
 */
bool udp_comm_send(const char *data, size_t len);

/**
 * @brief 注册 UDP 接收回调函数
 *
 * 每次收到 UDP 数据时，模块将调用已注册的回调。
 * 可在 udp_comm_init() 之前或之后调用。
 *
 * @param cb 回调函数指针，传 NULL 可注销回调
 */
void udp_comm_register_rx_callback(udp_rx_callback_t cb);

#ifdef __cplusplus
}
#endif
