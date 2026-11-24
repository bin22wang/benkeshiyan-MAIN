/**
 ****************************************************************************************************
 * @file        at24c02.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       AT24C02驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论�?:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "at24c02.h"

static bool s_i2c_inited = false;
static bool s_i2c_attempted = false;

/**
 * @brief       初始化AT24C02
 * @param       �?
 * @retval      ESP_OK:初始化成�?
 */
esp_err_t at24c02_init(void)
{
    if (s_i2c_inited) {
        return ESP_OK;
    }
    if (s_i2c_attempted) {
        return ESP_FAIL;
    }
    s_i2c_attempted = true;

    s_i2c_inited = true;
    return ESP_OK;
}

/**
 * @brief       在AT24C02指定地址读出一个数�?
 * @param       addr: 开始读数的地址
 * @retval      读到的数�?
 */
uint8_t at24c02_read_one_byte(uint8_t addr)
{
    uint8_t data = 0;

    if (at24c02_init() != ESP_OK) {
        return 0;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AT_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AT_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    (void)i2c_master_cmd_begin(AT24C02_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return data;
}

/**
 * @brief       在AT24C02指定地址写入一个数�?
 * @param       addr: 写入数据的目的地址
 * @param       data: 要写入的数据
 * @retval      �?
 */
void at24c02_write_one_byte(uint8_t addr, uint8_t data)
{
    uint8_t send_buf[2] = {0};

    if (at24c02_init() != ESP_OK) {
        return;
    }

    send_buf[0] = addr % 256;
    send_buf[1] = data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AT_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, send_buf, sizeof(send_buf), true);
    i2c_master_stop(cmd);
    (void)i2c_master_cmd_begin(AT24C02_I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    /* 由于AT24C02写入过慢，需延迟10ms左右时间 */
    esp_rom_delay_us(10000);
}

/**
 * @brief       检查AT24C02是否正常
 * @note        检测原�?: 在器件的末地址写如0X55, 然后再读�?, 如果读取值为0X55
 *              则表示检测正�?. 否则,则表示检测失�?.
 * @param       �?
 * @retval      检测结�?
 *              0: 检测成�?
 *              1: 检测失�?
 */
uint8_t at24c02_check(void)
{
    uint8_t temp;
    uint16_t addr = AT24C02;

    temp = at24c02_read_one_byte(addr);     /* 避免每次开机都写AT24CXX */

    if (temp == 0X55)                       /* 读取数据正常 */
    {
        return 0;
    }
    else                                    /* 排除第一次初始化的情�? */
    {
        at24c02_write_one_byte(addr, 0X55); /* 先写入数�? */
        temp = at24c02_read_one_byte(255);  /* 再读取数�? */

        if (temp == 0X55)
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief       在AT24C02里面的指定地址开始读出指定个数的数据
 * @param       addr    : 开始读出的地址 �?24c02�?0~255
 * @param       pbuf    : 数据数组首地址
 * @param       datalen : 要读出数据的个数
 * @retval      �?
 */
void at24c02_read(uint8_t addr, uint8_t *pbuf, uint8_t datalen)
{
    while (datalen--)
    {
        *pbuf++ = at24c02_read_one_byte(addr++);
    }
}

/**
 * @brief       在AT24C02里面的指定地址开始写入指定个数的数据
 * @param       addr    : 开始写入的地址 �?24c02�?0~255
 * @param       pbuf    : 数据数组首地址
 * @param       datalen : 要写入数据的个数
 * @retval      �?
 */
void at24c02_write(uint8_t addr, uint8_t *pbuf, uint8_t datalen)
{
    while (datalen--)
    {
        at24c02_write_one_byte(addr, *pbuf);
        addr++;
        pbuf++;
    }
}
