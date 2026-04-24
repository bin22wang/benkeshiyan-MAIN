#pragma once

#include <stdint.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "xl9555.h"

#define LCD_NUM_WR GPIO_NUM_40
#define LCD_NUM_CS GPIO_NUM_21

#define LCD_WR(x)  do { gpio_set_level(LCD_NUM_WR, (x) ? 1 : 0); } while (0)
#define LCD_CS(x)  do { gpio_set_level(LCD_NUM_CS, (x) ? 1 : 0); } while (0)
#define LCD_PWR(x) do { xl9555_pin_write(SLCD_PWR_IO, (x) ? 1 : 0); } while (0)
#define LCD_RST(x) do { xl9555_pin_write(SLCD_RST_IO, (x) ? 1 : 0); } while (0)

#define LCD_WIDTH  320
#define LCD_HEIGHT 240

/* Common colors (RGB565) */
#define WHITE   0xFFFF
#define BLACK   0x0000
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define CYAN    0x07FF
#define GRAY    0x8430

#define LCD_FONT_BASE_W   5
#define LCD_FONT_BASE_H   7
#define LCD_FONT_SPACING  1

#define LCD_FONT_SCALE(size)   (((size) / 8) < 1 ? 1 : ((size) / 8))
#define LCD_CHAR_W(size)       ((LCD_FONT_BASE_W + LCD_FONT_SPACING) * LCD_FONT_SCALE(size))
#define LCD_CHAR_H(size)       ((LCD_FONT_BASE_H + 1) * LCD_FONT_SCALE(size))

typedef struct {
    uint16_t width;
    uint16_t height;
    uint8_t dir;
    uint16_t wramcmd;
    uint16_t setxcmd;
    uint16_t setycmd;
    uint16_t wr;
    uint16_t cs;
} lcd_obj_t;

#define LCD_TOTAL_BUF_SIZE (LCD_WIDTH * LCD_HEIGHT * 2)

extern lcd_obj_t lcd_self;
extern uint8_t lcd_buf[LCD_TOTAL_BUF_SIZE];

void lcd_init(void);
void lcd_set_dir(uint8_t dir);
void lcd_write_cmd(uint8_t cmd);
void lcd_write_data(const uint8_t *data, int len);
void lcd_write_data16(uint16_t data);
void lcd_clear(uint16_t color);
void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color);
void lcd_show_char(uint16_t x, uint16_t y, char chr, uint8_t size, uint8_t mode, uint16_t color);
void lcd_show_string(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                     uint8_t size, char *p, uint16_t color);
void lcd_draw_rgb565(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                     const uint8_t *buf, size_t buf_len);
void lcd_set_invert(bool enable);
