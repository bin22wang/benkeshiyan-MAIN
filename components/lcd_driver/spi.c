#include "spi.h"

#include <string.h>

#include "esp_err.h"

static bool s_bus_inited = false;

esp_err_t spi2_init(void)
{
    if (s_bus_inited) {
        return ESP_OK;
    }

    spi_bus_config_t bus_cfg = {
        .miso_io_num = SPI_MISO_GPIO_PIN,
        .mosi_io_num = SPI_MOSI_GPIO_PIN,
        .sclk_io_num = SPI_CLK_GPIO_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320 * 240 * 2,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret == ESP_OK) {
        s_bus_inited = true;
    }
    return ret;
}

void spi2_write_cmd(spi_device_handle_t handle, uint8_t cmd)
{
    if (!handle) {
        return;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;

    ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &t));
}

void spi2_write_data(spi_device_handle_t handle, const uint8_t *data, int len)
{
    if (!handle || !data || len <= 0) {
        return;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = (size_t)len * 8;
    t.tx_buffer = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(handle, &t));
}

uint8_t spi2_transfer_byte(spi_device_handle_t handle, uint8_t data)
{
    if (!handle) {
        return 0;
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.tx_data[0] = data;

    ESP_ERROR_CHECK(spi_device_transmit(handle, &t));
    return t.rx_data[0];
}
