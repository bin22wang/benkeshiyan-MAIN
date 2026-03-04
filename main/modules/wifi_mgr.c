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
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Wi-Fi disconnected, retrying... (%d/%d)",
                     s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi connect failed after %d retries", WIFI_MAX_RETRY);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ========== 公共接口实现 ========== */

bool wifi_mgr_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Event group create failed, no memory");
        return false;
    }

    /*
     * esp_netif_init() and esp_event_loop_create_default()
     * are called in app_main.c. Only create STA netif here.
     */
    esp_netif_create_default_wifi_sta();

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

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init done, waiting for SSID: %s ...", WIFI_SSID);

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
