#include "calib_mgr.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "disp_mgr.h"
#include "xl9555.h"
#include "tof_ctrl.h"
#include "calib_store.h"

#define CALIB_POINT_COUNT 3
#define CALIB_FOCUS_EXIT 3
#define CALIB_SAMPLES 10
#define KEY_DEBOUNCE_MS 30
#define KEY_SCAN_MS 20

static const char *TAG = "calib_mgr";

static const float k_calib_dist_cm[CALIB_POINT_COUNT] = {8.0f, 30.0f, 70.0f};

static bool s_active = false;
static uint8_t s_focus = 0;
static uint8_t s_done_mask = 0;
static bool s_using_defaults = true;
static uint16_t s_left_raw[CALIB_POINT_COUNT] = {0};
static uint16_t s_right_raw[CALIB_POINT_COUNT] = {0};
static float s_left_scale = 1.0f;
static float s_left_bias = 0.0f;
static float s_right_scale = 1.0f;
static float s_right_bias = 0.0f;

static void calib_update_ui(void)
{
    disp_calib_state_t ui = {
        .focus = s_focus,
        .done_mask = s_done_mask,
        .using_defaults = s_using_defaults,
        .left_scale = s_left_scale,
        .left_bias = s_left_bias,
        .right_scale = s_right_scale,
        .right_bias = s_right_bias,
    };
    disp_mgr_update_calib(&ui);
}

static bool key_pressed(uint16_t key_mask)
{
    return xl9555_pin_read(key_mask) == 0;
}

static void wait_key_release(uint16_t key_mask)
{
    int guard = 50;
    while (key_pressed(key_mask) && guard-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t calib_collect_raw(uint16_t *left_mm, uint16_t *right_mm)
{
    uint32_t left_sum = 0;
    uint32_t right_sum = 0;
    uint8_t left_cnt = 0;
    uint8_t right_cnt = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        tof_dual_reading_t reading = {0};
        if (tof_ctrl_read_raw(&reading) == ESP_OK) {
            if (reading.left_ok) {
                left_sum += reading.left_mm;
                left_cnt++;
            }
            if (reading.right_ok) {
                right_sum += reading.right_mm;
                right_cnt++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (left_cnt == 0 || right_cnt == 0) {
        return ESP_FAIL;
    }

    *left_mm = (uint16_t)((left_sum + (left_cnt / 2)) / left_cnt);
    *right_mm = (uint16_t)((right_sum + (right_cnt / 2)) / right_cnt);
    return ESP_OK;
}

static bool calib_compute_line(const uint16_t *x_mm, const float *y_mm, const uint8_t *idx,
                               int count, float *scale, float *bias)
{
    if (count < 2) {
        return false;
    }

    if (count == 2) {
        float x1 = (float)x_mm[idx[0]];
        float x2 = (float)x_mm[idx[1]];
        float y1 = y_mm[idx[0]];
        float y2 = y_mm[idx[1]];
        float denom = x2 - x1;
        if (denom == 0.0f) {
            return false;
        }
        *scale = (y2 - y1) / denom;
        *bias = y1 - (*scale) * x1;
        return true;
    }

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_x2 = 0.0f;
    float sum_xy = 0.0f;
    for (int i = 0; i < count; i++) {
        float x = (float)x_mm[idx[i]];
        float y = y_mm[idx[i]];
        sum_x += x;
        sum_y += y;
        sum_x2 += x * x;
        sum_xy += x * y;
    }
    float denom = (count * sum_x2) - (sum_x * sum_x);
    if (denom == 0.0f) {
        return false;
    }
    *scale = ((count * sum_xy) - (sum_x * sum_y)) / denom;
    *bias = (sum_y - (*scale) * sum_x) / (float)count;
    return true;
}

static void calib_update_params(void)
{
    uint8_t idx[CALIB_POINT_COUNT];
    int count = 0;
    for (uint8_t i = 0; i < CALIB_POINT_COUNT; i++) {
        if (s_done_mask & (1U << i)) {
            idx[count++] = i;
        }
    }
    if (count < 2) {
        return;
    }

    float y_mm[CALIB_POINT_COUNT];
    for (int i = 0; i < CALIB_POINT_COUNT; i++) {
        y_mm[i] = k_calib_dist_cm[i] * 10.0f;
    }

    float left_scale = s_left_scale;
    float left_bias = s_left_bias;
    float right_scale = s_right_scale;
    float right_bias = s_right_bias;

    bool left_ok = calib_compute_line(s_left_raw, y_mm, idx, count, &left_scale, &left_bias);
    bool right_ok = calib_compute_line(s_right_raw, y_mm, idx, count, &right_scale, &right_bias);

    if (left_ok) {
        s_left_scale = left_scale;
        s_left_bias = left_bias;
    }
    if (right_ok) {
        s_right_scale = right_scale;
        s_right_bias = right_bias;
    }

    s_using_defaults = false;

    tof_ctrl_set_calibration(s_left_scale, s_left_bias, s_right_scale, s_right_bias);

    calib_params_t params = {
        .left_scale = s_left_scale,
        .left_bias = s_left_bias,
        .right_scale = s_right_scale,
        .right_bias = s_right_bias,
    };
    calib_store_save(&params);
    calib_update_ui();
}

static void calib_handle_action(void)
{
    if (s_focus == CALIB_FOCUS_EXIT) {
        if (s_done_mask != 0) {
            calib_params_t params = {
                .left_scale = s_left_scale,
                .left_bias = s_left_bias,
                .right_scale = s_right_scale,
                .right_bias = s_right_bias,
            };
            calib_store_save(&params);
        }
        disp_mgr_set_page(DISP_PAGE_STATUS);
        calib_mgr_set_active(false);
        return;
    }

    uint16_t left_mm = 0;
    uint16_t right_mm = 0;
    if (calib_collect_raw(&left_mm, &right_mm) != ESP_OK) {
        ESP_LOGW(TAG, "calib sample failed");
        return;
    }

    s_left_raw[s_focus] = left_mm;
    s_right_raw[s_focus] = right_mm;
    s_done_mask |= (uint8_t)(1U << s_focus);

    calib_update_params();
}

static void calib_task(void *pvParameters)
{
    (void)pvParameters;

    bool last_key1 = false;
    bool last_key2 = false;

    while (1) {
        if (!s_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        bool key1 = key_pressed(KEY1_IO);
        bool key2 = key_pressed(KEY2_IO);

        if (key1 && !last_key1) {
            s_focus = (uint8_t)((s_focus + 1) % (CALIB_POINT_COUNT + 1));
            calib_update_ui();
            wait_key_release(KEY1_IO);
        }
        if (key2 && !last_key2) {
            calib_handle_action();
            wait_key_release(KEY2_IO);
        }

        last_key1 = key1;
        last_key2 = key2;
        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_MS));
    }
}

esp_err_t calib_mgr_init(void)
{
    (void)xl9555_init();
    if (calib_store_init() != ESP_OK) {
        ESP_LOGW(TAG, "EEPROM init failed");
    }

    calib_params_t params = {0};
    if (calib_store_load(&params)) {
        s_left_scale = params.left_scale;
        s_left_bias = params.left_bias;
        s_right_scale = params.right_scale;
        s_right_bias = params.right_bias;
        s_using_defaults = false;
    } else {
        s_using_defaults = true;
    }
    tof_ctrl_set_calibration(s_left_scale, s_left_bias, s_right_scale, s_right_bias);
    calib_update_ui();
    return ESP_OK;
}

void calib_mgr_start(void)
{
    xTaskCreate(calib_task, "calib_task", 4096, NULL, 3, NULL);
}

void calib_mgr_set_active(bool active)
{
    s_active = active;
    if (s_active) {
        calib_update_ui();
    }
}

bool calib_mgr_is_active(void)
{
    return s_active;
}
