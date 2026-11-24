/**
 * @file udp_comm.c
 * @brief UDP 通信模块实现
 *
 * 封装了 UDP socket 的完整生命周期：
 *  - 创建 socket、绑定本地端口
 *  - 后台 FreeRTOS 任务持续接收数据
 *  - 通过回调机制将收到的数据通知上层
 *  - 提供发送接口（如需多任务并发发送，建议由上层加锁串行化）
 */

#include "udp_comm.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

/* ========== 模块日志标签 ========== */
static const char *TAG = "udp_comm";

/* ========== 接收缓冲区大小 ========== */
#define UDP_RX_BUF_SIZE   512

/* ========== 模块内部状态 ========== */

/** UDP socket 文件描述符，-1 表示未初始化 */
static int s_sock = -1;

/** 服务器地址结构体（发送目标） */
static struct sockaddr_in s_dest_addr;

/** 发送串行化锁，避免多个任务同时发送同一个 socket。 */
static SemaphoreHandle_t s_send_mutex = NULL;

/** 上层注册的接收回调函数 */
static udp_rx_callback_t s_rx_callback = NULL;

/* ========== 内部任务 ========== */

/**
 * @brief UDP 接收后台任务
 *
 * 在独立的 FreeRTOS 任务中持续调用 recvfrom()，
 * 收到数据后调用已注册的回调函数通知上层。
 * 使用 1 秒接收超时，避免任务永久阻塞。
 *
 * @param pvParameters 未使用
 */
static void udp_rx_task(void *pvParameters)
{
    char rxbuf[UDP_RX_BUF_SIZE];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    ESP_LOGI(TAG, "UDP RX task started");

    while (1) {
        /*
         * recvfrom():
         *  - 返回值 > 0：收到 UDP 数据
         *  - 返回值 = 0：UDP 一般不会出现（对端关闭无“连接”语义）
         *  - 返回值 < 0：出错；当设置了 SO_RCVTIMEO 时，超时会返回 -1 并置 errno=EAGAIN/EWOULDBLOCK
         */
        int rx_len = recvfrom(s_sock, rxbuf, sizeof(rxbuf) - 1, 0,
                              (struct sockaddr *)&source_addr, &socklen);

        if (rx_len > 0) {
            /* 追加 '\0' 方便作为 C 字符串打印日志；len 仍以 rx_len 为准 */
            rxbuf[rx_len] = '\0';

            char src_ip[16];
            /* inet_ntoa() 返回静态缓冲区指针，拷贝一份避免后续调用覆盖 */
            strncpy(src_ip, inet_ntoa(source_addr.sin_addr), sizeof(src_ip) - 1);
            uint16_t src_port = ntohs(source_addr.sin_port);

            ESP_LOGI(TAG, "RX <- %s:%u, len=%d, data=%s",
                     src_ip, src_port, rx_len, rxbuf);

            if (s_rx_callback != NULL) {
                /* 回调在本接收任务上下文执行：建议回调内保持轻量，必要时转发到其他队列/任务 */
                s_rx_callback(rxbuf, (size_t)rx_len, src_ip, src_port);
            }

        } else if (rx_len < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "recvfrom error: errno=%d", errno);
            }
        }
    }

    vTaskDelete(NULL);
}

/* ========== 公共接口实现 ========== */

bool udp_comm_init(const char *server_ip, uint16_t server_port,
                   uint16_t local_port)
{
    if (server_ip == NULL) {
        ESP_LOGE(TAG, "server_ip cannot be NULL");
        return false;
    }

    /*
     * 配置发送目标地址（服务器地址）：udp_comm_send() 会将数据发往此地址。
     * 注意：UDP 无连接，"server" 仅表示默认目的端。
     */
    memset(&s_dest_addr, 0, sizeof(s_dest_addr));
    s_dest_addr.sin_family      = AF_INET;
    s_dest_addr.sin_addr.s_addr = inet_addr(server_ip);
    s_dest_addr.sin_port        = htons(server_port);

    /* 创建 UDP socket */
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_sock < 0) {
        ESP_LOGE(TAG, "socket 创建失败: errno=%d", errno);
        return false;
    }

    /* 绑定本地端口，用于接收数据（INADDR_ANY 表示监听所有本机网卡） */
    struct sockaddr_in local_addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port        = htons(local_port),
    };

    if (bind(s_sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "bind port %u failed: errno=%d", local_port, errno);
        close(s_sock);
        s_sock = -1;
        return false;
    }

    /* 设置接收超时，避免 recvfrom 永久阻塞，便于循环内继续做错误处理等 */
    struct timeval timeout = {
        .tv_sec  = 1,
        .tv_usec = 0,
    };
    setsockopt(s_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(TAG, "UDP socket ready, server=%s:%u, local_port=%u",
             server_ip, server_port, local_port);

    /* 启动接收任务：后续收到 UDP 数据将通过回调通知上层 */
    BaseType_t ret = xTaskCreate(udp_rx_task, "udp_rx_task",
                                 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "RX task create failed");
        close(s_sock);
        s_sock = -1;
        return false;
    }

    if (s_send_mutex == NULL) {
        s_send_mutex = xSemaphoreCreateMutex();
        if (s_send_mutex == NULL) {
            ESP_LOGE(TAG, "send mutex create failed");
            close(s_sock);
            s_sock = -1;
            return false;
        }
    }

    return true;
}

bool udp_comm_send(const char *data, size_t len)
{
    if (s_sock < 0) {
        ESP_LOGE(TAG, "UDP not init, cannot send");
        return false;
    }

    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "send data empty");
        return false;
    }

    if (s_send_mutex == NULL || xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "send busy, drop packet");
        return false;
    }

    /* sendto(): 直接指定目的地址（s_dest_addr），无需 connect() */
    int err = sendto(s_sock, data, len, 0,
                     (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
    xSemaphoreGive(s_send_mutex);

    if (err < 0) {
        if (errno == ENOMEM || errno == ENOBUFS) {
            ESP_LOGW(TAG, "sendto failed: errno=%d", errno);
        } else {
            ESP_LOGE(TAG, "sendto failed: errno=%d", errno);
        }
        return false;
    }

    ESP_LOGI(TAG, "sent -> %s:%u, len=%d",
             inet_ntoa(s_dest_addr.sin_addr),
             ntohs(s_dest_addr.sin_port),
             err);
    return true;
}

void udp_comm_register_rx_callback(udp_rx_callback_t cb)
{
    s_rx_callback = cb;
    if (cb != NULL) {
        ESP_LOGI(TAG, "UDP RX callback registered");
    } else {
        ESP_LOGW(TAG, "UDP RX callback unregistered");
    }
}
