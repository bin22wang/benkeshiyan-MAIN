/**
 * @file led_ctrl.c
 * @brief LED status indicator for DNESP32S3 board
 *
 * DNESP32S3 board: LED cathode connected to GPIO1 (active LOW).
 * gpio_set_level(1, 0) = LED ON
 * gpio_set_level(1, 1) = LED OFF
 */

#include "led_ctrl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* ========== Config ========== */
#define LED_GPIO_NUM    GPIO_NUM_1  /* DNESP32S3: LED cathode on GPIO1 */
#define LED_ON          0           /* Active LOW: 0 = LED on  */
#define LED_OFF         1           /* Active LOW: 1 = LED off */

/* ========== Module tag ========== */
static const char *TAG = "led_ctrl";

/* ========== Internal state ========== */
static volatile led_mode_t s_led_mode = LED_MODE_DISCONNECTED;

/* ========== Background task ========== */

static void led_blink_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED blink task started, GPIO=%d (active LOW)", LED_GPIO_NUM);

    while (1) {
        switch (s_led_mode) {

            case LED_MODE_DISCONNECTED:
                /* Fast blink: 500ms on / 500ms off */
                gpio_set_level(LED_GPIO_NUM, LED_ON);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_GPIO_NUM, LED_OFF);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;

            case LED_MODE_CONNECTED:
                /* Slow blink: 1000ms on / 1000ms off */
                gpio_set_level(LED_GPIO_NUM, LED_ON);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_GPIO_NUM, LED_OFF);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case LED_MODE_ACTIVE:
                /* Solid on */
                gpio_set_level(LED_GPIO_NUM, LED_ON);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;

            default:
                gpio_set_level(LED_GPIO_NUM, LED_OFF);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
        }
    }

    vTaskDelete(NULL);
}

/* ========== Public API ========== */

void led_ctrl_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_NUM),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO_NUM, LED_OFF); /* off at start */

    ESP_LOGI(TAG, "LED GPIO %d init OK (active LOW)", LED_GPIO_NUM);

    xTaskCreate(led_blink_task, "led_blink_task", 2048, NULL, 2, NULL);
}

void led_ctrl_set_mode(led_mode_t mode)
{
    if (s_led_mode != mode) {
        s_led_mode = mode;

        const char *mode_str[] = {
            "DISCONNECTED (fast blink)",
            "CONNECTED (slow blink)",
            "ACTIVE (solid on)",
        };
        if ((int)mode < 3) {
            ESP_LOGI(TAG, "LED mode -> %s", mode_str[mode]);
        }
    }
}
