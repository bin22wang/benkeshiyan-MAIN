/**
 * @file video_streamer.h
 * @brief 高度模块化的 UDP 按需图传模块头文件
 *
 * 该模块实现了完全独立的视频流传输功能，与现有控制逻辑解耦：
 *  - 通过独立的 FreeRTOS 任务运行
 *  - 使用标准 UDP 接口（udp_comm_send）进行数据发送
 *  - 支持 JPEG 软件压缩与分片传输
 *  - 提供简洁的启停接口（video_streamer_start/stop）
 *
 * 协议规范（V 协议）：
 *  - 接收: [V,START,<CARn>] 开启图传；[V,STOP,<CARn>] 关闭图传
 *  - 发送元数据: [V,META,<CARn>,<序列号>,<JPEG总字节数>]
 *  - 发送分片: [V,CHUNK,<CARn>,<序列号>,<IDX>,<TOT>,<大小>]\n<二进制数据>
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化视频流传输模块
 *
 * 必须在 camera_ui 初始化完成、UDP 模块初始化完成后调用。
 * 该函数会：
 *  1. 初始化内部状态变量
 *  2. 创建视频流传输 FreeRTOS 任务（优先级中等）
 *  3. 任务初始处于"停止"状态，等待 start 信号
 *
 * @note 该函数可在 app_main.c 中的初始化阶段调用，或由上层按需调用。
 *       由于内部任务已启动，多次调用可能导致资源泄漏，建议仅调用一次。
 */
void video_streamer_init(const char *car_id);

/**
 * @brief 开启图传
 *
 * 将内部状态标志置为 "正在流传"，视频流任务开始：
 *  1. 循环获取摄像头画面（RGB565）
 *  2. 软件压缩为 JPEG（质量参数 30）
 *  3. 按 900 字节分片，通过 UDP 发送
 *  4. 帧率控制在 5~10 FPS
 *
 * @note 该函数是非阻塞的，仅标记状态。实际的流传控制由内部任务异步执行。
 *       可被重复调用而不产生额外任务。
 */
void video_streamer_start(void);

/**
 * @brief 关闭图传
 *
 * 将内部状态标志置为 "停止"，视频流任务进入低功耗等待。
 * 正在发送的帧会继续完成，然后任务进入 vTaskDelay 等待。
 *
 * @note 该函数是非阻塞的，仅标记状态。
 *       可被重复调用而不产生额外任务。
 */
void video_streamer_stop(void);

#ifdef __cplusplus
}
#endif
