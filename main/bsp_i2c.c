#include "bsp_i2c.h"

#include "driver/i2c.h"

#define BSP_I2C_PORT I2C_NUM_0
#define BSP_I2C_SDA_GPIO GPIO_NUM_39
#define BSP_I2C_SCL_GPIO GPIO_NUM_38
#define BSP_I2C_SPEED_HZ 400000

#define BSP_I2C1_PORT I2C_NUM_1
#define BSP_I2C1_SDA_GPIO GPIO_NUM_41
#define BSP_I2C1_SCL_GPIO GPIO_NUM_42
#define BSP_I2C1_SPEED_HZ 400000

esp_err_t bsp_i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA_GPIO,
        .scl_io_num = BSP_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_SPEED_HZ,
    };

    esp_err_t ret = i2c_param_config(BSP_I2C_PORT, &conf);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    ret = i2c_driver_install(BSP_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }

    return ret;
}

esp_err_t bsp_i2c_aux_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C1_SDA_GPIO,
        .scl_io_num = BSP_I2C1_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C1_SPEED_HZ,
    };

    esp_err_t ret = i2c_param_config(BSP_I2C1_PORT, &conf);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    ret = i2c_driver_install(BSP_I2C1_PORT, conf.mode, 0, 0, 0);
    if (ret == ESP_ERR_INVALID_STATE) {
        ret = ESP_OK;
    }

    return ret;
}
