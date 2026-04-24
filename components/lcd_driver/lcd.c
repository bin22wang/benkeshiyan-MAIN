#include "lcd.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#include "spi.h"

#ifndef LCD_X_OFFSET
#define LCD_X_OFFSET 0
#endif

#ifndef LCD_Y_OFFSET
#define LCD_Y_OFFSET 0
#endif

lcd_obj_t lcd_self = {
    .width = LCD_WIDTH,
    .height = LCD_HEIGHT,
    .dir = 1,
    .wramcmd = 0x2C,
    .setxcmd = 0x2A,
    .setycmd = 0x2B,
    .wr = LCD_NUM_WR,
    .cs = LCD_NUM_CS,
};

uint8_t lcd_buf[LCD_TOTAL_BUF_SIZE];

static spi_device_handle_t s_lcd_handle = NULL;
static uint16_t s_bg_color = BLACK;
static SemaphoreHandle_t s_lcd_mutex = NULL;

static void lcd_mutex_init(void)
{
    if (s_lcd_mutex == NULL) {
        s_lcd_mutex = xSemaphoreCreateRecursiveMutex();
    }
}

static void lcd_lock(void)
{
    if (s_lcd_mutex != NULL) {
        xSemaphoreTakeRecursive(s_lcd_mutex, portMAX_DELAY);
    }
}

static void lcd_unlock(void)
{
    if (s_lcd_mutex != NULL) {
        xSemaphoreGiveRecursive(s_lcd_mutex);
    }
}


static const uint8_t k_font_digits[10][LCD_FONT_BASE_H] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},
    {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E},
    {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E},
};

static const uint8_t k_font_upper[26][LCD_FONT_BASE_H] = {
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F},
    {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C},
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},
    {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D},
    {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E},
    {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11},
    {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04},
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},
};

static const uint8_t k_font_space[LCD_FONT_BASE_H] = {0};
static const uint8_t k_font_colon[LCD_FONT_BASE_H] = {0x00, 0x04, 0x00, 0x00, 0x04, 0x00, 0x00};
static const uint8_t k_font_dot[LCD_FONT_BASE_H] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00};
static const uint8_t k_font_minus[LCD_FONT_BASE_H] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
static const uint8_t k_font_comma[LCD_FONT_BASE_H] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08};

static const uint8_t *lcd_get_glyph_rows(char c)
{
    if (c >= '0' && c <= '9') {
        return k_font_digits[c - '0'];
    }

    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }

    if (c >= 'A' && c <= 'Z') {
        return k_font_upper[c - 'A'];
    }

    switch (c) {
        case ' ': return k_font_space;
        case ':': return k_font_colon;
        case '.': return k_font_dot;
        case '-': return k_font_minus;
        case ',': return k_font_comma;
        default:  return k_font_space;
    }
}

static void lcd_gpio_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << LCD_NUM_WR) | (1ULL << LCD_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);

    LCD_WR(1);
    LCD_CS(1);
}

void lcd_write_cmd(uint8_t cmd)
{
    if (!s_lcd_handle) {
        return;
    }

    lcd_lock();
    LCD_CS(0);
    LCD_WR(0);
    spi2_write_cmd(s_lcd_handle, cmd);
    LCD_CS(1);
    lcd_unlock();
}

void lcd_write_data(const uint8_t *data, int len)
{
    if (!s_lcd_handle || !data || len <= 0) {
        return;
    }

    lcd_lock();
    LCD_CS(0);
    LCD_WR(1);
    spi2_write_data(s_lcd_handle, data, len);
    LCD_CS(1);
    lcd_unlock();
}

void lcd_write_data16(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFF);
    lcd_write_data(buf, 2);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];
    uint16_t xs = (uint16_t)(x0 + LCD_X_OFFSET);
    uint16_t xe = (uint16_t)(x1 + LCD_X_OFFSET);
    uint16_t ys = (uint16_t)(y0 + LCD_Y_OFFSET);
    uint16_t ye = (uint16_t)(y1 + LCD_Y_OFFSET);

    lcd_write_cmd((uint8_t)lcd_self.setxcmd);
    data[0] = (uint8_t)(xs >> 8);
    data[1] = (uint8_t)(xs & 0xFF);
    data[2] = (uint8_t)(xe >> 8);
    data[3] = (uint8_t)(xe & 0xFF);
    lcd_write_data(data, 4);

    lcd_write_cmd((uint8_t)lcd_self.setycmd);
    data[0] = (uint8_t)(ys >> 8);
    data[1] = (uint8_t)(ys & 0xFF);
    data[2] = (uint8_t)(ye >> 8);
    data[3] = (uint8_t)(ye & 0xFF);
    lcd_write_data(data, 4);

    lcd_write_cmd((uint8_t)lcd_self.wramcmd);
}

static void lcd_write_color_repeat(uint16_t color, size_t count)
{
    uint8_t line[64 * 2];
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    for (size_t i = 0; i < 64; ++i) {
        line[i * 2] = hi;
        line[i * 2 + 1] = lo;
    }

    while (count > 0) {
        size_t batch = (count > 64) ? 64 : count;
        spi2_write_data(s_lcd_handle, line, (int)(batch * 2));
        count -= batch;
    }
}

static void lcd_draw_bitmap(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                            const uint16_t *colors)
{
    if (!colors || x1 < x0 || y1 < y0) {
        return;
    }

    uint16_t width = (uint16_t)(x1 - x0 + 1);
    uint16_t height = (uint16_t)(y1 - y0 + 1);

    lcd_set_window(x0, y0, x1, y1);

    uint8_t *line = (uint8_t *)malloc(width * 2);
    if (!line) {
        return;
    }

    LCD_CS(0);
    LCD_WR(1);

    for (uint16_t row = 0; row < height; ++row) {
        const uint16_t *src = colors + (row * width);
        for (uint16_t col = 0; col < width; ++col) {
            uint16_t c = src[col];
            line[col * 2] = (uint8_t)(c >> 8);
            line[col * 2 + 1] = (uint8_t)(c & 0xFF);
        }
        spi2_write_data(s_lcd_handle, line, (int)(width * 2));
    }

    LCD_CS(1);
    free(line);
}

void lcd_set_dir(uint8_t dir)
{
    lcd_lock();
    lcd_self.dir = dir ? 1 : 0;
    if (lcd_self.dir) {
        lcd_self.width = LCD_WIDTH;
        lcd_self.height = LCD_HEIGHT;
    } else {
        lcd_self.width = LCD_HEIGHT;
        lcd_self.height = LCD_WIDTH;
    }

    uint8_t madctl = lcd_self.dir ? 0x60 : 0x00;
    lcd_write_cmd(0x36);
    lcd_write_data(&madctl, 1);
    lcd_unlock();
}

void lcd_set_invert(bool enable)
{
    lcd_write_cmd(enable ? 0x21 : 0x20);
}

void lcd_init(void)
{
    lcd_mutex_init();
    lcd_lock();
    ESP_ERROR_CHECK(spi2_init());
    lcd_gpio_init();

    if (!s_lcd_handle) {
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = 60 * 1000 * 1000,
            .mode = 0,
            .spics_io_num = -1,
            .queue_size = 1,
        };
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_lcd_handle));
    }

    ESP_ERROR_CHECK(xl9555_init());
    LCD_PWR(1);
    vTaskDelay(pdMS_TO_TICKS(10));
    LCD_RST(0);
    vTaskDelay(pdMS_TO_TICKS(20));
    LCD_RST(1);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_write_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_write_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_write_cmd(0x3A);
    {
        uint8_t data = 0x55;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xB2);
    {
        uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
        lcd_write_data(data, (int)sizeof(data));
    }

    lcd_write_cmd(0xB7);
    {
        uint8_t data = 0x35;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xBB);
    {
        uint8_t data = 0x2B;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xC0);
    {
        uint8_t data = 0x2C;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xC2);
    {
        uint8_t data = 0x01;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xC3);
    {
        uint8_t data = 0x11;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xC4);
    {
        uint8_t data = 0x20;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xC6);
    {
        uint8_t data = 0x0F;
        lcd_write_data(&data, 1);
    }

    lcd_write_cmd(0xD0);
    {
        uint8_t data[] = {0xA4, 0xA1};
        lcd_write_data(data, (int)sizeof(data));
    }

    lcd_write_cmd(0xE0);
    {
        uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
        lcd_write_data(data, (int)sizeof(data));
    }

    lcd_write_cmd(0xE1);
    {
        uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
        lcd_write_data(data, (int)sizeof(data));
    }

    lcd_write_cmd(0x20);

    lcd_set_dir(1);
    lcd_write_cmd(0x29);
    vTaskDelay(pdMS_TO_TICKS(20));

    lcd_clear(BLACK);
    lcd_unlock();
}

void lcd_clear(uint16_t color)
{
    lcd_lock();
    s_bg_color = color;
    lcd_fill_rect(0, 0, (uint16_t)(lcd_self.width - 1), (uint16_t)(lcd_self.height - 1), color);
    lcd_unlock();
}

void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    if (x1 < x0 || y1 < y0) {
        return;
    }

    if (x0 >= lcd_self.width || y0 >= lcd_self.height) {
        return;
    }

    if (x1 >= lcd_self.width) {
        x1 = (uint16_t)(lcd_self.width - 1);
    }
    if (y1 >= lcd_self.height) {
        y1 = (uint16_t)(lcd_self.height - 1);
    }

    lcd_lock();
    lcd_set_window(x0, y0, x1, y1);

    LCD_CS(0);
    LCD_WR(1);
    lcd_write_color_repeat(color, (size_t)(x1 - x0 + 1) * (size_t)(y1 - y0 + 1));
    LCD_CS(1);
    lcd_unlock();
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= lcd_self.width || y >= lcd_self.height) {
        return;
    }

    lcd_lock();
    lcd_set_window(x, y, x, y);
    LCD_CS(0);
    LCD_WR(1);
    lcd_write_color_repeat(color, 1);
    LCD_CS(1);
    lcd_unlock();
}

void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color)
{
    lcd_lock();
    uint16_t scale = LCD_FONT_SCALE(size);
    uint16_t char_w = LCD_CHAR_W(size);
    uint16_t char_h = LCD_CHAR_H(size);
    const uint8_t *glyph = lcd_get_glyph_rows(chr);

    if (mode != 0) {
        for (uint16_t row = 0; row < LCD_FONT_BASE_H; ++row) {
            uint8_t row_bits = glyph[row];
            for (uint16_t col = 0; col < LCD_FONT_BASE_W; ++col) {
                if (row_bits & (1U << (LCD_FONT_BASE_W - 1 - col))) {
                    for (uint16_t dy = 0; dy < scale; ++dy) {
                        for (uint16_t dx = 0; dx < scale; ++dx) {
                            lcd_draw_point((uint16_t)(x + col * scale + dx),
                                           (uint16_t)(y + row * scale + dy),
                                           color);
                        }
                    }
                }
            }
        }
        return;
    }

    size_t total = (size_t)char_w * (size_t)char_h;
    uint16_t *buf = (uint16_t *)malloc(total * sizeof(uint16_t));
    if (!buf) {
        return;
    }

    for (uint16_t row = 0; row < char_h; ++row) {
        uint16_t src_row = (uint16_t)(row / scale);
        uint8_t row_bits = (src_row < LCD_FONT_BASE_H) ? glyph[src_row] : 0;
        for (uint16_t col = 0; col < char_w; ++col) {
            uint16_t src_col = (uint16_t)(col / scale);
            bool on = false;
            if (src_col < LCD_FONT_BASE_W) {
                on = (row_bits & (1U << (LCD_FONT_BASE_W - 1 - src_col))) != 0;
            }
            buf[row * char_w + col] = on ? color : s_bg_color;
        }
    }

    lcd_draw_bitmap(x, y, (uint16_t)(x + char_w - 1), (uint16_t)(y + char_h - 1), buf);
    free(buf);
    lcd_unlock();
}

void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                     uint8_t size, char *p, uint16_t color)
{
    if (!p) {
        return;
    }

    lcd_lock();
    uint16_t x0 = x;
    uint16_t char_w = LCD_CHAR_W(size);
    uint16_t char_h = LCD_CHAR_H(size);
    uint16_t max_x = (uint16_t)(x + width);
    uint16_t max_y = (uint16_t)(y + height);

    while (*p) {
        if ((uint16_t)(x + char_w) > max_x) {
            x = x0;
            y = (uint16_t)(y + char_h);
        }
        if ((uint16_t)(y + char_h) > max_y) {
            break;
        }

        lcd_show_char(x, y, *p, size, 0, color);
        x = (uint16_t)(x + char_w);
        ++p;
    }
    lcd_unlock();
}

void lcd_draw_rgb565(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                     const uint8_t *buf, size_t buf_len)
{
    if (!buf || width == 0 || height == 0) {
        return;
    }

    if (x >= lcd_self.width || y >= lcd_self.height) {
        return;
    }

    if ((uint32_t)x + width > lcd_self.width) {
        width = (uint16_t)(lcd_self.width - x);
    }
    if ((uint32_t)y + height > lcd_self.height) {
        height = (uint16_t)(lcd_self.height - y);
    }

    size_t row_bytes = (size_t)width * 2U;
    size_t expected = row_bytes * (size_t)height;
    if (buf_len > 0 && buf_len < expected) {
        return;
    }

    uint8_t *dma_line = (uint8_t *)heap_caps_malloc(row_bytes, MALLOC_CAP_DMA);
    if (!dma_line) {
        return;
    }

    lcd_lock();
    lcd_set_window(x, y, (uint16_t)(x + width - 1), (uint16_t)(y + height - 1));

    LCD_CS(0);
    LCD_WR(1);

    for (uint16_t row = 0; row < height; ++row) {
        const uint8_t *src = buf + ((size_t)row * row_bytes);
        memcpy(dma_line, src, row_bytes);
        spi2_write_data(s_lcd_handle, dma_line, (int)row_bytes);
    }

    LCD_CS(1);
    heap_caps_free(dma_line);
    lcd_unlock();
}
