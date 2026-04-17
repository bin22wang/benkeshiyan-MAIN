#include "tof_ctrl.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "vl53l0x_api.h"

#define TOF_I2C_PORT I2C_NUM_0
#define TOF_I2C_SDA_GPIO GPIO_NUM_5
#define TOF_I2C_SCL_GPIO GPIO_NUM_4
#define TOF_I2C_SPEED_HZ 400000

#define TOF_XSHUT_LEFT_GPIO  GPIO_NUM_7
#define TOF_XSHUT_RIGHT_GPIO GPIO_NUM_6

#define TOF_ADDR_DEFAULT 0x29
#define TOF_ADDR_LEFT    0x30
#define TOF_ADDR_RIGHT   0x31

static const char *TAG = "tof_ctrl";

static VL53L0X_Dev_t s_left_dev = {0};
static VL53L0X_Dev_t s_right_dev = {0};
static bool s_inited = false;

static tof_dual_reading_t s_last_reading = {0};
static bool s_last_valid = false;
static uint32_t s_last_update_ms = 0;

#define TOF_FILTER_WINDOW 5

typedef struct {
    uint16_t buf[TOF_FILTER_WINDOW];
    uint8_t count;
    uint8_t index;
} tof_filter_t;

static tof_filter_t s_left_filter = {0};
static tof_filter_t s_right_filter = {0};

static float s_left_scale = 1.0f;
static float s_left_bias = 0.0f;
static float s_right_scale = 1.0f;
static float s_right_bias = 0.0f;

static uint16_t tof_apply_calibration(float scale, float bias, uint16_t raw_mm)
{
    float calibrated = scale * (float)raw_mm + bias;
    if (calibrated < 0.0f) {
        calibrated = 0.0f;
    }
    return (uint16_t)(calibrated + 0.5f);
}

static uint16_t tof_filter_apply(tof_filter_t *filt, uint16_t sample)
{
    filt->buf[filt->index] = sample;
    filt->index = (uint8_t)((filt->index + 1) % TOF_FILTER_WINDOW);
    if (filt->count < TOF_FILTER_WINDOW) {
        filt->count++;
    }

    uint16_t temp[TOF_FILTER_WINDOW];
    for (uint8_t i = 0; i < filt->count; i++) {
        temp[i] = filt->buf[i];
    }

    for (uint8_t i = 1; i < filt->count; i++) {
        uint16_t key = temp[i];
        int j = (int)i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    uint16_t median = temp[filt->count / 2];

    uint32_t sum = 0;
    for (uint8_t i = 0; i < filt->count; i++) {
        sum += temp[i];
    }
    uint16_t avg = (uint16_t)((sum + (filt->count / 2)) / filt->count);

    return (uint16_t)((median + avg) / 2);
}

static esp_err_t tof_read_once(tof_dual_reading_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    VL53L0X_RangingMeasurementData_t left_data = {0};
    VL53L0X_RangingMeasurementData_t right_data = {0};

    VL53L0X_Error left_status = VL53L0X_PerformSingleRangingMeasurement(&s_left_dev, &left_data);
    VL53L0X_Error right_status = VL53L0X_PerformSingleRangingMeasurement(&s_right_dev, &right_data);

    out->left_mm = left_data.RangeMilliMeter;
    out->right_mm = right_data.RangeMilliMeter;
    out->left_ok = (left_status == VL53L0X_ERROR_NONE) && (left_data.RangeStatus == 0);
    out->right_ok = (right_status == VL53L0X_ERROR_NONE) && (right_data.RangeStatus == 0);

    return (left_status == VL53L0X_ERROR_NONE || right_status == VL53L0X_ERROR_NONE)
               ? ESP_OK
               : ESP_FAIL;
}

static void tof_xshut_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << TOF_XSHUT_LEFT_GPIO) | (1ULL << TOF_XSHUT_RIGHT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    gpio_set_level(TOF_XSHUT_LEFT_GPIO, 0);
    gpio_set_level(TOF_XSHUT_RIGHT_GPIO, 0);
}

static VL53L0X_Error tof_device_basic_init(VL53L0X_Dev_t *dev)
{
    VL53L0X_Error status;
    uint8_t vhv = 0;
    uint8_t phase = 0;
    uint32_t ref_spad_count = 0;
    uint8_t is_aperture_spads = 0;

    status = VL53L0X_DataInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_StaticInit(dev);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_PerformRefCalibration(dev, &vhv, &phase);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(dev, &ref_spad_count, &is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    return status;
}

static VL53L0X_Error tof_set_address(VL53L0X_Dev_t *dev, uint8_t new_addr)
{
    VL53L0X_Error status = VL53L0X_SetDeviceAddress(dev, (uint8_t)(new_addr << 1));
    if (status == VL53L0X_ERROR_NONE) {
        dev->i2c_address = new_addr;
    }
    return status;
}

static esp_err_t tof_init_one(VL53L0X_Dev_t *dev, uint8_t new_addr)
{
    VL53L0X_Error status = tof_device_basic_init(dev);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "device init failed: %d", (int)status);
        return ESP_FAIL;
    }

    status = tof_set_address(dev, new_addr);
    if (status != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "set addr 0x%02X failed: %d", new_addr, (int)status);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tof_ctrl_init(void)
{
    esp_err_t ret;

    tof_xshut_init();
    gpio_set_level(TOF_XSHUT_LEFT_GPIO, 0);
    gpio_set_level(TOF_XSHUT_RIGHT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    s_left_dev.i2c_port_num = TOF_I2C_PORT;
    s_left_dev.i2c_address = TOF_ADDR_DEFAULT;
    s_right_dev.i2c_port_num = TOF_I2C_PORT;
    s_right_dev.i2c_address = TOF_ADDR_DEFAULT;

    gpio_set_level(TOF_XSHUT_LEFT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = tof_init_one(&s_left_dev, TOF_ADDR_LEFT);
    if (ret != ESP_OK) {
        return ret;
    }

    gpio_set_level(TOF_XSHUT_RIGHT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = tof_init_one(&s_right_dev, TOF_ADDR_RIGHT);
    if (ret != ESP_OK) {
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "ToF init done (0x%02X, 0x%02X)", TOF_ADDR_LEFT, TOF_ADDR_RIGHT);
    return ESP_OK;
}

esp_err_t tof_ctrl_read(tof_dual_reading_t *out)
{
    esp_err_t ret = tof_read_once(out);
    if (ret != ESP_OK) {
        return ret;
    }

    if (out->left_ok) {
        out->left_mm = tof_apply_calibration(s_left_scale, s_left_bias, out->left_mm);
        out->left_mm = tof_filter_apply(&s_left_filter, out->left_mm);
    }
    if (out->right_ok) {
        out->right_mm = tof_apply_calibration(s_right_scale, s_right_bias, out->right_mm);
        out->right_mm = tof_filter_apply(&s_right_filter, out->right_mm);
    }

    s_last_reading = *out;
    s_last_valid = true;
    s_last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    return ESP_OK;
}

esp_err_t tof_ctrl_read_raw(tof_dual_reading_t *out)
{
    esp_err_t ret = tof_read_once(out);
    if (ret == ESP_OK && out) {
        s_last_reading = *out;
        s_last_valid = true;
        s_last_update_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }
    return ret;
}

esp_err_t tof_ctrl_get_latest(tof_dual_reading_t *out, uint32_t *age_ms)
{
    if (!out || !s_last_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    *out = s_last_reading;
    if (age_ms) {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        *age_ms = now - s_last_update_ms;
    }
    return ESP_OK;
}

void tof_ctrl_set_calibration(float left_scale, float left_bias,
                              float right_scale, float right_bias)
{
    s_left_scale = left_scale;
    s_left_bias = left_bias;
    s_right_scale = right_scale;
    s_right_bias = right_bias;
}

void tof_ctrl_get_calibration(float *left_scale, float *left_bias,
                              float *right_scale, float *right_bias)
{
    if (left_scale) {
        *left_scale = s_left_scale;
    }
    if (left_bias) {
        *left_bias = s_left_bias;
    }
    if (right_scale) {
        *right_scale = s_right_scale;
    }
    if (right_bias) {
        *right_bias = s_right_bias;
    }
}
