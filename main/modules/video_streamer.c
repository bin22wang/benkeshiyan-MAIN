/**
 * @file video_streamer.c
 * @brief 高度模块化的 UDP 按需图传模块实现
 *
 * 核心特性：
 *  - 独立 FreeRTOS 任务（video_stream_task）实现流传输逻辑
 *  - RGB565 → JPEG 软件压缩（frame2jpg）
 *  - 自适应分片传输（最大 900 字节/片）
 *  - 防拥堵延时机制（每片间 vTaskDelay(2ms)）
 *  - 完全解耦：不依赖 recon_mgr、不干涉控制逻辑
 */

#include "video_streamer.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"

#include "udp_comm.h"

/* ========================================================================
 * 模块配置与常量
 * ======================================================================*/

static const char *TAG = "video_streamer";

/** 最大 UDP 负载大小（为了保证传输效率，JPEG 分片不超过 900 字节） */
#define MAX_CHUNK_SIZE     900

/** 任务栈大小（字节） */
#define TASK_STACK_SIZE    (4 * 1024)

/** 任务优先级（中等） */
#define TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

/** JPEG 压缩质量参数（0~100，30 为中等压缩） */
#define JPEG_QUALITY       30

/** 无流传时的任务休眠时间（毫秒） */
#define IDLE_SLEEP_MS      100

/** 帧间隔目标（毫秒，对应 10 FPS） */
#define FRAME_INTERVAL_MS  100

/** 分片间延时（毫秒，防止 UDP 队列拥堵） */
#define CHUNK_DELAY_MS     2

/** 序列号最多为 16 位（用于追踪 JPEG 帧） */
#define SEQ_NUM_MAX        0xFFFF

/* ========================================================================
 * 内部状态变量
 * ======================================================================*/

/** 是否启用了流传输 */
static volatile bool s_streaming_enabled = false;

/** 本模块对应的小车 ID（用于协议包标识） */
static char s_streamer_car_id[16] = {0};

/** 当前 JPEG 序列号（每发完一帧 +1）*/
static uint16_t s_seq_num = 0;

/** 任务句柄（便于后续管理任务生命周期） */
static TaskHandle_t s_task_handle = NULL;

/** 发送互斥锁（保护对 udp_comm_send 的并发访问） */
static SemaphoreHandle_t s_send_mutex = NULL;

/* ========================================================================
 * 内部工具函数
 * ======================================================================*/

/**
 * @brief 安全地发送 UDP 数据（加锁保护）
 *
 * @param data 数据指针
 * @param len  数据长度
 * @return true  发送成功
 * @return false 发送失败或超时获取锁
 */
static bool send_udp_safe(const char *data, size_t len)
{
    if (s_send_mutex == NULL || !xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG, "failed to acquire send mutex");
        return false;
    }

    bool ret = udp_comm_send(data, len);

    xSemaphoreGive(s_send_mutex);
    return ret;
}

/**
 * @brief 发送 META 包（元数据）
 *
 * 格式: [V,META,<car_id>,<序列号>,<JPEG总字节数>]
 *
 * @param seq_num 当前 JPEG 序列号
 * @param jpg_len 当前 JPEG 总大小（字节）
 * @return true  发送成功
 * @return false 发送失败
 */
static bool send_meta_packet(uint16_t seq_num, size_t jpg_len)
{
    /* 构造 META 包，格式: [V,META,<car_id>,<seq>,<len>] */
    char meta_buf[64];
    int len = snprintf(meta_buf, sizeof(meta_buf), "[V,META,%s,%u,%zu]",
                       s_streamer_car_id, seq_num, jpg_len);
    if (len <= 0 || len >= (int)sizeof(meta_buf)) {
        ESP_LOGE(TAG, "META packet buffer overflow");
        return false;
    }

    ESP_LOGD(TAG, "-> META: %s", meta_buf);
    return send_udp_safe(meta_buf, (size_t)len);
}

static bool send_chunk_packet(uint32_t seq_num, uint32_t chunk_idx, uint32_t total_chunks, const uint8_t *payload, size_t payload_len)
{
    char header_buf[128];
    // 1. 生成文本头
    int header_len = snprintf(header_buf, sizeof(header_buf), "[V,CHUNK,%s,%lu,%lu,%lu,%zu]\n",
                              s_streamer_car_id, (unsigned long)seq_num,
                              (unsigned long)chunk_idx, (unsigned long)total_chunks, payload_len);

    // 2. 申请内存拼接
    size_t packet_size = header_len + payload_len;
    uint8_t *packet = (uint8_t *)malloc(packet_size);
    if (packet == NULL) {
        return false; // 内存申请失败，返回 false
    }
    
    memcpy(packet, header_buf, header_len);                  // 放入文本头
    memcpy(packet + header_len, payload, payload_len);       // 紧接着放入图像二进制

    // 3. 一次性发送，并记录发送结果
    bool ret = udp_comm_send((const char *)packet, packet_size);
    
    free(packet); // 释放内存
    
    return ret;   // 返回发送成功与否
}

/**
 * @brief 分片并发送一个完整的 JPEG 帧
 *
 * 流程:
 *  1. 发送 META 包
 *  2. 按 MAX_CHUNK_SIZE 分片，逐片发送
 *  3. 每片间延时 CHUNK_DELAY_MS 以防拥堵
 *
 * @param seq_num 序列号
 * @param jpg_buf JPEG 数据缓冲区
 * @param jpg_len JPEG 总大小（字节）
 * @return true  全部发送成功
 * @return false 任意发送失败则立即返回
 */
static bool send_jpeg_frame(uint16_t seq_num, const uint8_t *jpg_buf,
                            size_t jpg_len)
{
    if (!jpg_buf || jpg_len == 0) {
        return false;
    }

    /* 第 1 步：发送 META 包 */
    if (!send_meta_packet(seq_num, jpg_len)) {
        ESP_LOGW(TAG, "failed to send META packet for seq=%u", seq_num);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(CHUNK_DELAY_MS));

    /* 第 2 步：计算分片总数 */
    uint16_t total_chunks = (uint16_t)((jpg_len + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE);

    /* 第 3 步：逐片发送 */
    for (uint16_t chunk_idx = 0; chunk_idx < total_chunks; chunk_idx++) {
        size_t offset = (size_t)chunk_idx * MAX_CHUNK_SIZE;
        size_t remaining = jpg_len - offset;
        size_t chunk_size = (remaining > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : remaining;

        if (!send_chunk_packet(seq_num, chunk_idx, total_chunks,
                               jpg_buf + offset, chunk_size)) {
            ESP_LOGW(TAG, "failed to send chunk %u/%u for seq=%u",
                     chunk_idx, total_chunks, seq_num);
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(CHUNK_DELAY_MS));
    }

    ESP_LOGI(TAG, "frame sent: seq=%u, size=%zu, chunks=%u",
             seq_num, jpg_len, total_chunks);

    return true;
}

/* ========================================================================
 * 核心流传任务
 * ======================================================================*/

/**
 * @brief 视频流传输主任务
 *
 * 循环逻辑:
 *  1. 若未启用 → sleep(100ms) → 继续检查
 *  2. 若已启用：
 *     a. 获取摄像头画面（RGB565）
 *     b. 软件压缩为 JPEG
 *     c. 分片发送（分片间加延时）
 *     d. 释放内存
 *     e. 帧间延时以控制帧率
 */
static void video_stream_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "video_stream_task started");

    while (1) {
        /* 检查是否需要流传输 */
        if (!s_streaming_enabled) {
            vTaskDelay(pdMS_TO_TICKS(IDLE_SLEEP_MS));
            continue;
        }

        /* ---- 已启用流传输 ---- */

        /* 1. 获取摄像头画面缓冲区（高优先级，获取最新帧） */
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            ESP_LOGW(TAG, "failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(IDLE_SLEEP_MS));
            continue;
        }

        /* 2. 转换为 JPEG 格式 */
        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;

        if (!frame2jpg(fb, JPEG_QUALITY, &jpg_buf, &jpg_len)) {
            ESP_LOGW(TAG, "failed to compress frame to JPEG");
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(IDLE_SLEEP_MS));
            continue;
        }

        /* 3. 释放硬件摄像头缓冲区 */
        esp_camera_fb_return(fb);

        /* 4. 分片发送 */
        if (jpg_len > 0 && jpg_buf != NULL) {
            send_jpeg_frame(s_seq_num, jpg_buf, jpg_len);

            /* 升级序列号 */
            s_seq_num++;
            if (s_seq_num > SEQ_NUM_MAX) {
                s_seq_num = 0;
            }
        }

        /* 5. 释放 JPEG 缓冲区 */
        if (jpg_buf != NULL) {
            free(jpg_buf);
            jpg_buf = NULL;
        }

        /* 6. 帧间延时，控制帧率 5~10 FPS */
        vTaskDelay(pdMS_TO_TICKS(FRAME_INTERVAL_MS));
    }

    vTaskDelete(NULL);
}

/* ========================================================================
 * 公开 API
 * ======================================================================*/

void video_streamer_init(const char *car_id)
{
    if (s_task_handle != NULL) {
        ESP_LOGW(TAG, "already initialized");
        return;
    }

    ESP_LOGI(TAG, "initializing...");

    if (car_id == NULL) {
        car_id = "";
    }
    strncpy(s_streamer_car_id, car_id, sizeof(s_streamer_car_id) - 1);
    s_streamer_car_id[sizeof(s_streamer_car_id) - 1] = '\0';

    if (s_streamer_car_id[0] == '\0') {
        ESP_LOGW(TAG, "empty car_id passed to video_streamer_init");
    }

    /* 创建发送互斥锁 */
    s_send_mutex = xSemaphoreCreateMutex();
    if (s_send_mutex == NULL) {
        ESP_LOGE(TAG, "failed to create send mutex");
        return;
    }

    /* 初始化状态 */
    s_streaming_enabled = false;
    s_seq_num = 0;

    /* 创建流传输任务 */
    BaseType_t ret = xTaskCreate(video_stream_task,
                                 "video_stream",
                                 TASK_STACK_SIZE,
                                 NULL,
                                 TASK_PRIORITY,
                                 &s_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create video_stream task");
        return;
    }

    ESP_LOGI(TAG, "initialized successfully");
}

void video_streamer_start(void)
{
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "not initialized yet, call video_streamer_init() first");
        return;
    }

    s_streaming_enabled = true;
    ESP_LOGI(TAG, "video streaming started");
}

void video_streamer_stop(void)
{
    s_streaming_enabled = false;
    ESP_LOGI(TAG, "video streaming stopped");
}
