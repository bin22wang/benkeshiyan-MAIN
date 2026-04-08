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

static uint16_t tof_apply_calibration(float scale, float bias, uint16_t raw_mm)
{
    float calibrated = scale * (float)raw_mm + bias;
    if (calibrated < 0.0f) {
        calibrated = 0.0f;
    }
    return (uint16_t)(calibrated + 0.5f);
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

static esp_err_t tof_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOF_I2C_SDA_GPIO,
        .scl_io_num = TOF_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOF_I2C_SPEED_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(TOF_I2C_PORT, &conf));
    esp_err_t ret = i2c_driver_install(TOF_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return ret;
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
    esp_err_t ret = tof_i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c init failed: %d", (int)ret);
        return ret;
    }

    tof_xshut_init();
    vTaskDelay(pdMS_TO_TICKS(20));

    s_left_dev.i2c_port_num = TOF_I2C_PORT;
    s_left_dev.i2c_address = TOF_ADDR_DEFAULT;
    s_right_dev.i2c_port_num = TOF_I2C_PORT;
    s_right_dev.i2c_address = TOF_ADDR_DEFAULT;

    gpio_set_level(TOF_XSHUT_LEFT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ret = tof_init_one(&s_left_dev, TOF_ADDR_LEFT);
    if (ret != ESP_OK) {
        return ret;
    }

    gpio_set_level(TOF_XSHUT_RIGHT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

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
        out->left_mm = tof_apply_calibration(0.963f, -28.1f, out->left_mm);
    }
    if (out->right_ok) {
        out->right_mm = tof_apply_calibration(0.997f, -27.3f, out->right_mm);
    }

    return ESP_OK;
}

esp_err_t tof_ctrl_read_raw(tof_dual_reading_t *out)
{
    return tof_read_once(out);
}
