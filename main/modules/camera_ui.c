#include "camera_ui.h"

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"

#include "disp_mgr.h"
#include "lcd.h"
#include "xl9555.h"

static const char *TAG = "camera_ui";

#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   -1
#define CAM_PIN_SIOD   -1
#define CAM_PIN_SIOC   -1
#define CAM_PIN_D7     18
#define CAM_PIN_D6     17
#define CAM_PIN_D5     16
#define CAM_PIN_D4     15
#define CAM_PIN_D3     7
#define CAM_PIN_D2     6
#define CAM_PIN_D1     5
#define CAM_PIN_D0     4
#define CAM_PIN_VSYNC  47
#define CAM_PIN_HREF   48
#define CAM_PIN_PCLK   45

#define CAM_XCLK_FREQ_HZ 20000000
#define CAM_UI_TOP 36

static camera_config_t s_cam_cfg = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = CAM_XCLK_FREQ_HZ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, 
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 20,
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,
    .sccb_i2c_port = I2C_NUM_0,
};

static void camera_hw_reset(void)
{
    xl9555_pin_write(OV_PWDN_IO, 1);
    xl9555_pin_write(OV_RESET_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    xl9555_pin_write(OV_PWDN_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    xl9555_pin_write(OV_RESET_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void camera_ui_task(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        if (disp_mgr_get_page() != DISP_PAGE_CAMERA) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera frame buffer get failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (disp_mgr_get_page() == DISP_PAGE_CAMERA) {
            if (fb->format == PIXFORMAT_RGB565) {
                uint16_t draw_y = CAM_UI_TOP;
                uint16_t draw_h = 0;

                if (fb->height > CAM_UI_TOP) {
                    draw_h = (uint16_t)(fb->height - CAM_UI_TOP);
                }

                if (draw_h > 0) {
                    const uint8_t *src = fb->buf + ((size_t)fb->width * CAM_UI_TOP * 2U);
                    size_t src_len = fb->len - ((size_t)fb->width * CAM_UI_TOP * 2U);
                    lcd_draw_rgb565(0, draw_y, (uint16_t)fb->width, draw_h, src, src_len);
                }
            }
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

bool camera_ui_init(void)
{
    if (xl9555_init() != ESP_OK) {
        ESP_LOGE(TAG, "XL9555 init failed");
        return false;
    }

    camera_hw_reset();

    esp_err_t ret = esp_camera_init(&s_cam_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(ret));
        return false;
    }

    xTaskCreatePinnedToCore(camera_ui_task, "camera_ui_task", 4096, NULL, 4, NULL, 1);
    return true;
}
