#include "xl9555.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "xl9555";

static bool s_i2c_inited = false;

static esp_err_t xl9555_ensure_bus(void)
{
    if (s_i2c_inited) {
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = XL9555_I2C_SDA_GPIO,
        .scl_io_num = XL9555_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = XL9555_I2C_SPEED_HZ,
    };

    esp_err_t ret = i2c_param_config(XL9555_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(XL9555_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }
    if (ret == ESP_OK) {
        s_i2c_inited = true;
    }
    return ret;
}

static esp_err_t xl9555_ensure_device(void)
{
    esp_err_t ret = xl9555_ensure_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bus init failed: %d", (int)ret);
    }
    return ret;
}

esp_err_t xl9555_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = xl9555_ensure_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "device init failed: %d", (int)ret);
        return ret;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (XL9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (XL9555_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(XL9555_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t xl9555_write_bytes(uint8_t reg, const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = xl9555_ensure_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "device init failed: %d", (int)ret);
        return ret;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (XL9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    for (size_t i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(XL9555_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void xl9555_ioconfig(uint16_t config_value)
{
    uint8_t data[2];
    data[0] = (uint8_t)(config_value & 0xFF);
    data[1] = (uint8_t)((config_value >> 8) & 0xFF);

    esp_err_t err;
    do {
        err = xl9555_write_bytes(XL9555_CONFIG_PORT0_REG, data, sizeof(data));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "config 0x%04X failed: %d", config_value, (int)err);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } while (err != ESP_OK);
}

uint16_t xl9555_pin_write(uint16_t pin, int val)
{
    uint8_t data[2] = {0};
    if (xl9555_read_bytes(XL9555_OUTPUT_PORT0_REG, data, sizeof(data)) != ESP_OK) {
        return 0;
    }

    if (pin <= 0x0080) {
        if (val) {
            data[0] |= (uint8_t)(pin & 0xFF);
        } else {
            data[0] &= (uint8_t)~(pin & 0xFF);
        }
    } else {
        if (val) {
            data[1] |= (uint8_t)((pin >> 8) & 0xFF);
        } else {
            data[1] &= (uint8_t)~((pin >> 8) & 0xFF);
        }
    }

    if (xl9555_write_bytes(XL9555_OUTPUT_PORT0_REG, data, sizeof(data)) != ESP_OK) {
        return 0;
    }

    return (uint16_t)((data[1] << 8) | data[0]);
}

int xl9555_pin_read(uint16_t pin)
{
    uint8_t data[2] = {0};
    if (xl9555_read_bytes(XL9555_INPUT_PORT0_REG, data, sizeof(data)) != ESP_OK) {
        return 0;
    }

    uint16_t value = (uint16_t)((data[1] << 8) | data[0]);
    return (value & pin) ? 1 : 0;
}

esp_err_t xl9555_init(void)
{
    esp_err_t ret = xl9555_ensure_device();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "device init failed: %d", (int)ret);
        return ret;
    }

    uint8_t data[2] = {0};
    (void)xl9555_read_bytes(XL9555_INPUT_PORT0_REG, data, sizeof(data));

    xl9555_ioconfig(0xF003);
    xl9555_pin_write(BEEP_IO, 1);
    xl9555_pin_write(SPK_EN_IO, 1);

    return ESP_OK;
}
