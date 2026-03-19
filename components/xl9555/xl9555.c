#include "xl9555.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "xl9555";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;

static esp_err_t xl9555_ensure_bus(void)
{
    if (s_bus) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = XL9555_I2C_PORT,
        .scl_io_num = XL9555_I2C_SCL_GPIO,
        .sda_io_num = XL9555_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

static esp_err_t xl9555_ensure_device(void)
{
    if (s_dev) {
        return ESP_OK;
    }

    esp_err_t ret = xl9555_ensure_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bus init failed: %d", (int)ret);
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = XL9555_ADDR,
        .scl_speed_hz = XL9555_I2C_SPEED_HZ,
    };

    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %d", (int)ret);
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

    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, -1);
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

    uint8_t *buf = (uint8_t *)malloc(len + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    buf[0] = reg;
    memcpy(buf + 1, data, len);

    ret = i2c_master_transmit(s_dev, buf, len + 1, -1);
    free(buf);
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
