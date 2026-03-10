/**
 * @file wifi_mgr.c
 * @brief Wi-Fi STA 连接管理模块实现
 *
 * 封装了 ESP-IDF Wi-Fi STA 初始化流程，包含：
 *  - 事件回调处理（连接/断开/获取IP）
 *  - 自动重连（最多 WIFI_MAX_RETRY 次）
 *  - 事件组同步等待 IP
 */

#include "wifi_mgr.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

/* ========== 配置宏 ========== */

/** 目标 AP 的 SSID */
#define WIFI_SSID        "Xiaomi_1010"
/** 目标 AP 的密码 */
#define WIFI_PASSWORD    "12345678"
/** 最大重连次数，超过后置 FAIL 标志并放弃 */
#define WIFI_MAX_RETRY   10

/*
 * 说明：此处使用宏硬编码 SSID/密码仅用于示例/调试。
 * 若用于实际项目，建议将凭据放入 NVS（或通过配网/配置文件注入），避免明文固化在固件中。
 */

/* ========== 事件组位定义 ========== */

/** Wi-Fi 已连接并获取 IP 的事件位 */
#define WIFI_CONNECTED_BIT  BIT0
/** Wi-Fi 连接失败的事件位 */
#define WIFI_FAIL_BIT       BIT1

/* ========== 模块内部变量 ========== */

static const char *TAG = "wifi_mgr";

/** 用于任务间同步的事件组句柄 */
static EventGroupHandle_t s_wifi_event_group = NULL;

/** 当前重连次数计数器 */
static int s_retry_num = 0;

/* ========== 内部函数 ========== */

/**
 * @brief Wi-Fi 和 IP 事件回调函数
 *
 * 处理三种事件：
 *  - WIFI_EVENT_STA_START       : Wi-Fi 启动完成，发起连接
 *  - WIFI_EVENT_STA_DISCONNECTED: 断线，根据重连次数决定是否重连
 *  - IP_EVENT_STA_GOT_IP        : 成功获取 IP，置连接成功位
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /*
     * 回调触发上下文：由 ESP-IDF 默认事件循环任务调用（非中断）。
     * 注意保持处理逻辑轻量，避免在回调中做长耗时阻塞操作。
     */
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        /* Wi-Fi 驱动启动完成：发起第一次连接（后续是否成功由事件继续驱动） */
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        /*
         * 断线事件：可能由信号弱、AP 重启、密码错误等引起。
         * 这里按重试次数自动重连，超过上限则置 FAIL 位，唤醒等待方。
         */
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Wi-Fi disconnected, retrying... (%d/%d)",
                     s_retry_num, WIFI_MAX_RETRY);
        } else {
            /* 连接失败：通知等待 wifi_mgr_init() 的任务结束等待 */
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi connect failed after %d retries", WIFI_MAX_RETRY);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        /*
         * 成功获取 DHCP 分配的 IPv4 地址：此时网络已可用，可进行 socket/UDP 通信。
         * event_data 指向 ip_event_got_ip_t。
         */
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        /* 置位连接成功标志，唤醒等待方 */
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ========== 公共接口实现 ========== */

bool wifi_mgr_init(void)
{
    /* 事件组用于同步：wifi_event_handler() 置位，wifi_mgr_init() 等待 */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Event group create failed, no memory");
        return false;
    }

    /*
     * 前置条件：esp_netif_init() 与 esp_event_loop_create_default() 由上层调用（见 app_main.c）。
     * 这里仅创建默认 STA netif，并挂接 Wi-Fi 驱动到 TCP/IP 栈。
     */
    esp_netif_create_default_wifi_sta();

    /* 使用默认配置初始化 Wi-Fi 驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, &instance_any_id));

    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, &instance_got_ip));

    /*
     * STA 配置：设置认证门限/PMF，并填充 SSID/密码。
     * strncpy 使用 sizeof-1，确保不写越界；若长度刚好等于上限，末尾会保留 '\0' 空间。
     */
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable  = true,
                .required = false,
            },
        },
    };
    strncpy((char *)wifi_config.sta.ssid,
            WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password,
            WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

        /* 设置为 STA 模式并写入配置，然后启动 Wi-Fi 驱动 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done, waiting for SSID: %s ...", WIFI_SSID);

    /*
     * 阻塞等待：
     *  - WIFI_CONNECTED_BIT：成功获取 IP
     *  - WIFI_FAIL_BIT     ：超过最大重试次数仍失败
     */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected, SSID: %s", WIFI_SSID);
        return true;
    }

    ESP_LOGE(TAG, "Wi-Fi connect failed, SSID: %s", WIFI_SSID);
    return false;
}
