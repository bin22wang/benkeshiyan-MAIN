#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"

#ifndef XL9555_I2C_PORT
#define XL9555_I2C_PORT I2C_NUM_1
#endif

#ifndef XL9555_I2C_SPEED_HZ
#define XL9555_I2C_SPEED_HZ 400000
#endif

#ifndef XL9555_I2C_SDA_GPIO
#define XL9555_I2C_SDA_GPIO GPIO_NUM_41
#endif

#ifndef XL9555_I2C_SCL_GPIO
#define XL9555_I2C_SCL_GPIO GPIO_NUM_42
#endif

#define XL9555_ADDR 0x20

#define XL9555_INPUT_PORT0_REG      0x00
#define XL9555_INPUT_PORT1_REG      0x01
#define XL9555_OUTPUT_PORT0_REG     0x02
#define XL9555_OUTPUT_PORT1_REG     0x03
#define XL9555_INVERSION_PORT0_REG  0x04
#define XL9555_INVERSION_PORT1_REG  0x05
#define XL9555_CONFIG_PORT0_REG     0x06
#define XL9555_CONFIG_PORT1_REG     0x07

#define AP_INT_IO   0x0001
#define QMA_INT_IO  0x0002
#define SPK_EN_IO   0x0004
#define BEEP_IO     0x0008
#define OV_PWDN_IO  0x0010
#define OV_RESET_IO 0x0020
#define GBC_LED_IO  0x0040
#define GBC_KEY_IO  0x0080
#define LCD_BL_IO   0x0100
#define CT_RST_IO   0x0200
#define SLCD_RST_IO 0x0400
#define SLCD_PWR_IO 0x0800
#define KEY3_IO     0x1000
#define KEY2_IO     0x2000
#define KEY1_IO     0x4000
#define KEY0_IO     0x8000

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t xl9555_init(void);
esp_err_t xl9555_read_bytes(uint8_t reg, uint8_t *data, size_t len);
esp_err_t xl9555_write_bytes(uint8_t reg, const uint8_t *data, size_t len);
uint16_t xl9555_pin_write(uint16_t pin, int val);
int xl9555_pin_read(uint16_t pin);
void xl9555_ioconfig(uint16_t config_value);

#ifdef __cplusplus
}
#endif
