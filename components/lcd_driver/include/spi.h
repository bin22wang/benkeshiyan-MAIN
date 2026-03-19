#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

/* SPI2 pin definitions */
#define SPI_MOSI_GPIO_PIN GPIO_NUM_11
#define SPI_CLK_GPIO_PIN  GPIO_NUM_12
#define SPI_MISO_GPIO_PIN GPIO_NUM_13

esp_err_t spi2_init(void);
void spi2_write_cmd(spi_device_handle_t handle, uint8_t cmd);
void spi2_write_data(spi_device_handle_t handle, const uint8_t *data, int len);
uint8_t spi2_transfer_byte(spi_device_handle_t handle, uint8_t data);
