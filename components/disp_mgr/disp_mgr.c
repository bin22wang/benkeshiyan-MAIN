/**
 * @file disp_mgr.c
 * @brief Cyber-Mecha HUD Implementation
 */

#include "disp_mgr.h"

#include "lcd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

#ifndef DARKBLUE
#define DARKBLUE 0x01CF
#endif

#define UI_GREEN 0xE007
#define UI_RED   0x00F8

typedef struct {
    char car_id[16];
    bool wifi_connected;
    sys_mode_t mode;
    float x;
    float y;
    float yaw;
    uint32_t last_rx_ms;
} disp_state_t;

static disp_state_t s_current_state = {0};

static char s_last_car_id[16] = {0};
static char s_last_wlan[12] = {0};
static char s_last_center[16] = {0};
static char s_last_x[16] = {0};
static char s_last_y[16] = {0};
static char s_last_yaw[16] = {0};
static uint16_t s_last_wlan_color = 0;

static void hud_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    lcd_draw_point(x, y, color);
}

static void hud_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int dx = (int)x2 - (int)x1;
    int dy = (int)y2 - (int)y1;
    int sx = (dx >= 0) ? 1 : -1;
    int sy = (dy >= 0) ? 1 : -1;
    dx = dx >= 0 ? dx : -dx;
    dy = dy >= 0 ? dy : -dy;

    int err = (dx > dy ? dx : -dy) / 2;

    while (1) {
        hud_draw_point(x1, y1, color);
        if (x1 == x2 && y1 == y2) {
            break;
        }
        int e2 = err;
        if (e2 > -dx) {
            err -= dy;
            x1 = (uint16_t)((int)x1 + sx);
        }
        if (e2 < dy) {
            err += dx;
            y1 = (uint16_t)((int)y1 + sy);
        }
    }
}

static void hud_fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    lcd_fill_rect(x0, y0, x1, y1, color);
}

static bool hud_text_changed(char *last, size_t last_size, const char *current)
{
    if (strncmp(last, current, last_size) == 0) {
        return false;
    }
    strncpy(last, current, last_size - 1);
    last[last_size - 1] = '\0';
    return true;
}

static void hud_draw_text_region(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                 uint8_t size, const char *text, uint16_t color)
{
    hud_fill(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1), BLACK);
    lcd_show_string(x, y, w, h, size, (char *)text, color);
}

// Draw the static borders and texts only once to save CPU time
static void draw_static_ui_framework(void)
{
    const uint16_t max_x = (uint16_t)(LCD_WIDTH - 1);
    const uint16_t max_y = (uint16_t)(LCD_HEIGHT - 1);

    lcd_clear(BLACK);

    // Top Bar (Y: 0~35)
    hud_draw_line(0, 35, max_x, 35, GRAY);
    hud_draw_line(0, 36, max_x, 36, DARKBLUE);
    hud_fill(0, 0, 6, 35, CYAN); // Left accent block
    hud_fill(8, 8, max_x, 26, BLACK); // Clear top bar content area

    // Middle Target Area (Y: 37~140)
    // Corners brackets
    hud_draw_line(10, 45, 30, 45, GRAY);  hud_draw_line(10, 45, 10, 65, GRAY);
    hud_draw_line(290, 45, 310, 45, GRAY); hud_draw_line(310, 45, 310, 65, GRAY);
    hud_draw_line(10, 130, 30, 130, GRAY); hud_draw_line(10, 110, 10, 130, GRAY);
    hud_draw_line(290, 130, 310, 130, GRAY); hud_draw_line(310, 110, 310, 130, GRAY);

    // Bottom Telemetry Area (Y: 141~240)
    hud_draw_line(0, 140, max_x, 140, GRAY);
    hud_draw_line(180, 140, 180, max_y, DARKBLUE); // Vertical separator

    // Static labels (bold X/Y)
    lcd_show_string(12, 150, 36, 24, 24, "X:", GRAY);
    lcd_show_string(13, 150, 36, 24, 24, "X:", GRAY);
    lcd_show_string(12, 190, 36, 24, 24, "Y:", GRAY);
    lcd_show_string(13, 190, 36, 24, 24, "Y:", GRAY);
    lcd_show_string(190, 155, 40, 16, 16, "HDG:", GRAY);

}

static void hud_draw_car_id(const char *car_id)
{
    const uint16_t x = 4;
    const uint16_t y = 10;
    const uint16_t w = 108;
    const uint16_t h = 16;
    const uint16_t cw = LCD_CHAR_W(16);

    const char *digits = car_id;
    if (strncmp(car_id, "CAR", 3) == 0 && car_id[3] != '\0') {
        digits = car_id + 3;
    }

    hud_fill(x, y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1), BLACK);

    const char *prefix = "[ CAR ";
    lcd_show_string(x, y, w, h, 16, (char *)prefix, WHITE);

    uint16_t digits_x = (uint16_t)(x + cw * strlen(prefix));
    lcd_show_string(digits_x, y, w, h, 16, (char *)digits, WHITE);
    lcd_show_string((uint16_t)(digits_x + 1), y, w, h, 16, (char *)digits, WHITE);

    uint16_t suffix_x = (uint16_t)(digits_x + cw * strlen(digits));
    lcd_show_string(suffix_x, y, w, h, 16, " ]", WHITE);
}

static void disp_refresh_task(void *pvParameters)
{
    char str_buf[32];
    uint32_t current_time;
    bool is_synced;

    (void)pvParameters;
    draw_static_ui_framework();

    while (1) {
        current_time = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        is_synced = (current_time - s_current_state.last_rx_ms) < 2000;

        // === ZONE 1: Identity & Uplink (Top Bar) ===
        if (hud_text_changed(s_last_car_id, sizeof(s_last_car_id), s_current_state.car_id)) {
            hud_draw_car_id(s_current_state.car_id);
        }

        const char *wlan_text = s_current_state.wifi_connected ? "WLAN:OK" : "WLAN:ERR";
        uint16_t wlan_color = s_current_state.wifi_connected ? UI_GREEN : UI_RED;
        if (hud_text_changed(s_last_wlan, sizeof(s_last_wlan), wlan_text)
            || wlan_color != s_last_wlan_color) {
            hud_draw_text_region(200, 10, 96, 16, 16, wlan_text, wlan_color);
            s_last_wlan_color = wlan_color;
        }

        hud_fill(112, 8, 199, 26, BLACK);

        // === ZONE 2: Tactical Mode (Center) ===
        {
            const uint16_t center_y = 68;
            const uint16_t center_h = 40;
            const char *center_text = "OFFLINE";
            uint16_t center_x = 55;

            if (is_synced) {
                switch (s_current_state.mode) {
                    case SYS_MODE_IDLE:
                        center_text = "IDLE";
                        center_x = 100;
                        break;
                    case SYS_MODE_LEADER:
                        center_text = "LEAD";
                        center_x = 100;
                        break;
                    case SYS_MODE_FOLLOW:
                        center_text = "FOLL";
                        center_x = 100;
                        break;
                    case SYS_MODE_MANUAL:
                        center_text = "CTRL";
                        center_x = 100;
                        break;
                    default:
                        center_text = "UNKN";
                        center_x = 100;
                        break;
                }
            }

            if (hud_text_changed(s_last_center, sizeof(s_last_center), center_text)) {
                hud_fill(40, 68, 280, 108, BLACK);
                lcd_show_string(center_x, center_y, 200, center_h, 40, (char *)center_text, GRAY);
                lcd_show_string((uint16_t)(center_x + 1), center_y, 200, center_h, 40, (char *)center_text, GRAY);
            }
        }

        // === ZONE 3: Telemetry Data (Bottom) ===
        snprintf(str_buf, sizeof(str_buf), "%+06.2f", s_current_state.x);
        if (hud_text_changed(s_last_x, sizeof(s_last_x), str_buf)) {
            hud_draw_text_region(50, 150, 108, 24, 24, str_buf, WHITE);
        }

        snprintf(str_buf, sizeof(str_buf), "%+06.2f", s_current_state.y);
        if (hud_text_changed(s_last_y, sizeof(s_last_y), str_buf)) {
            hud_draw_text_region(50, 190, 108, 24, 24, str_buf, WHITE);
        }

        snprintf(str_buf, sizeof(str_buf), "%05.1f", s_current_state.yaw);
        if (hud_text_changed(s_last_yaw, sizeof(s_last_yaw), str_buf)) {
            hud_draw_text_region(225, 185, 90, 24, 24, str_buf, YELLOW);
        }

        // Run at 20 FPS for smooth updates
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void disp_mgr_init(void)
{
    lcd_init();
    lcd_set_dir(1); // Landscape
    LCD_PWR(1);

    strncpy(s_current_state.car_id, "INIT", sizeof(s_current_state.car_id) - 1);
    s_current_state.car_id[sizeof(s_current_state.car_id) - 1] = '\0';
    s_current_state.wifi_connected = false;
    s_current_state.mode = SYS_MODE_IDLE;
    s_current_state.last_rx_ms = 0; // Starts as LOST until data arrives

    xTaskCreate(disp_refresh_task, "disp_task", 4096, NULL, 3, NULL);
}

void disp_mgr_update_status(const char *car_id, bool wifi_connected,
                            sys_mode_t mode, float x, float y, float yaw)
{
    if (car_id && car_id[0] != '\0') {
        strncpy(s_current_state.car_id, car_id, sizeof(s_current_state.car_id) - 1);
        s_current_state.car_id[sizeof(s_current_state.car_id) - 1] = '\0';
    }
    s_current_state.wifi_connected = wifi_connected;
    s_current_state.mode = mode;
    s_current_state.x = x;
    s_current_state.y = y;
    s_current_state.yaw = yaw;
    s_current_state.last_rx_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
