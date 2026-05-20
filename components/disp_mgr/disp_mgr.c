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
#ifndef RGB565_GREEN
#define RGB565_GREEN GREEN
#endif

typedef struct {
    char car_id[16];
    bool wifi_connected;
    sys_mode_t mode;
    float x;
    float y;
    float yaw;
    uint32_t last_rx_ms;
    float aruco_x;
    float aruco_y;
    float aruco_yaw;
    float aruco_conf;
    bool aruco_valid;
    disp_recon_state_t recon_state;
    char recon_order[4][10];
    int recon_order_count;
} disp_state_t;

static disp_state_t s_current_state = {0};

static disp_page_t s_current_page = DISP_PAGE_STATUS;
static bool s_page_dirty = true;

static char s_last_car_id[16] = {0};
static char s_last_wlan[12] = {0};
static char s_last_center[16] = {0};
static char s_last_x[16] = {0};
static char s_last_y[16] = {0};
static char s_last_yaw[16] = {0};
static uint16_t s_last_wlan_color = 0;

static char s_last_cam_x[16] = {0};
static char s_last_cam_y[16] = {0};
static char s_last_cam_yaw[16] = {0};
static char s_last_cam_conf[16] = {0};
static char s_last_recon_state[16] = {0};
static char s_last_recon_order[64] = {0};
static char s_last_recon_owner[16] = {0};


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

static void hud_reset_cache(void)
{
    s_last_car_id[0] = '\0';
    s_last_wlan[0] = '\0';
    s_last_center[0] = '\0';
    s_last_x[0] = '\0';
    s_last_y[0] = '\0';
    s_last_yaw[0] = '\0';
    s_last_wlan_color = 0;
    s_last_cam_x[0] = '\0';
    s_last_cam_y[0] = '\0';
    s_last_cam_yaw[0] = '\0';
    s_last_cam_conf[0] = '\0';
    s_last_recon_state[0] = '\0';
    s_last_recon_order[0] = '\0';
    s_last_recon_owner[0] = '\0';
}

// Draw the static borders and texts only once to save CPU time
static void draw_status_ui_framework(void)
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

static void draw_recon_ui_framework(void)
{
    const uint16_t max_x = (uint16_t)(LCD_WIDTH - 1);
    const uint16_t max_y = (uint16_t)(LCD_HEIGHT - 1);

    lcd_clear(BLACK);
    hud_fill(0, 0, 6, 34, CYAN);
    hud_draw_line(0, 34, max_x, 34, GRAY);
    hud_draw_line(0, 35, max_x, 35, DARKBLUE);
    lcd_show_string(12, 8, 80, 16, 16, "RECON", WHITE);
    hud_draw_line(10, 52, max_x - 10, 52, GRAY);
    hud_draw_line(10, 53, max_x - 10, 53, DARKBLUE);
    hud_draw_line(10, 186, max_x - 10, 186, GRAY);
    hud_draw_line(10, 187, max_x - 10, 187, DARKBLUE);
    hud_draw_line(10, 54, 10, 186, GRAY);
    hud_draw_line(max_x - 10, 54, max_x - 10, 186, GRAY);
    hud_draw_line(0, 220, max_x, 220, GRAY);
    hud_draw_line(0, 221, max_x, 221, DARKBLUE);
}

static const char *recon_state_text(disp_recon_state_t state)
{
    switch (state) {
        case DISP_RECON_IDLE: return "IDLE";
        case DISP_RECON_PREPARING: return "PREPARING";
        case DISP_RECON_READY: return "READY";
        case DISP_RECON_ASSEMBLING: return "ASSEMBLING";
        case DISP_RECON_GUIDING: return "GUIDING!";
        case DISP_RECON_SUCCESS: return "SUCCESS";
        default: return "UNKN";
    }
}

static void recon_label_from_car(const char *car_id, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    if (!car_id || car_id[0] == '\0') {
        strncpy(out, "R?", out_len - 1);
        out[out_len - 1] = '\0';
        return;
    }
    if (strncmp(car_id, "CAR", 3) == 0 && car_id[3] != '\0') {
        snprintf(out, out_len, "R%s", car_id + 3);
        return;
    }
    snprintf(out, out_len, "R%s", car_id);
}

static void draw_recon_sequence(void)
{
    char summary[64] = {0};
    char label[12] = {0};
    size_t used = 0;
    const uint16_t bar_y = 10;
    const uint16_t bar_h = 16;
    const uint16_t cw = LCD_CHAR_W(16);

    if (s_current_state.recon_order_count <= 0) {
        strncpy(summary, "NO ORDER", sizeof(summary) - 1);
    } else {
        for (int i = 0; i < s_current_state.recon_order_count; i++) {
            recon_label_from_car(s_current_state.recon_order[i], label, sizeof(label));
            size_t remain = sizeof(summary) - used - 1;
            if (remain == 0) break;
            if (i > 0) {
                strncat(summary, " - ", remain);
                used = strlen(summary);
                remain = sizeof(summary) - used - 1;
            }
            strncat(summary, label, remain);
            used = strlen(summary);
        }
    }

    bool owner_changed = hud_text_changed(s_last_recon_owner, sizeof(s_last_recon_owner),
                                          s_current_state.car_id);
    if (hud_text_changed(s_last_recon_order, sizeof(s_last_recon_order), summary) || owner_changed) {
        hud_fill(8, 6, (uint16_t)(LCD_WIDTH - 9), 30, BLACK);
    }

    if (s_current_state.recon_order_count <= 0) {
        uint16_t x = (uint16_t)((LCD_WIDTH - (uint16_t)(strlen(summary) * cw)) / 2U);
        if (x < 12) x = 12;
        lcd_show_string(x, bar_y, 296, bar_h, 16, summary, GRAY);
        return;
    }

    uint16_t total_w = 0;
    for (int i = 0; i < s_current_state.recon_order_count; i++) {
        recon_label_from_car(s_current_state.recon_order[i], label, sizeof(label));
        total_w = (uint16_t)(total_w + (uint16_t)(strlen(label) * cw));
        if (i < s_current_state.recon_order_count - 1) {
            total_w = (uint16_t)(total_w + (uint16_t)(3 * cw));
        }
    }

    uint16_t x = 12;
    if (total_w < LCD_WIDTH) {
        x = (uint16_t)((LCD_WIDTH - total_w) / 2U);
        if (x < 12) x = 12;
    }
    uint16_t y = bar_y;
    for (int i = 0; i < s_current_state.recon_order_count; i++) {
        recon_label_from_car(s_current_state.recon_order[i], label, sizeof(label));
        uint16_t color = WHITE;
        if (s_current_state.car_id[0] != '\0' &&
            strcmp(s_current_state.recon_order[i], s_current_state.car_id) == 0) {
            color = RGB565_GREEN;
        }
        lcd_show_string(x, y, 120, bar_h, 16, label, color);
        x = (uint16_t)(x + cw * strlen(label));
        if (i < s_current_state.recon_order_count - 1) {
            lcd_show_string(x, y, 40, bar_h, 16, " - ", GRAY);
            x = (uint16_t)(x + cw * 3);
        }
    }
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
    draw_status_ui_framework();

    while (1) {
        if (s_page_dirty) {
            hud_reset_cache();
            if (s_current_page == DISP_PAGE_STATUS) {
                draw_status_ui_framework();
            } else if (s_current_page == DISP_PAGE_RECON) {
                draw_recon_ui_framework();
            } else {
                lcd_clear(BLACK);
            }
            s_page_dirty = false;
        }

        current_time = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        is_synced = (current_time - s_current_state.last_rx_ms) < 2000;

        if (s_current_page == DISP_PAGE_STATUS) {
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
        } else if (s_current_page == DISP_PAGE_RECON) {
            const char *state_text = recon_state_text(s_current_state.recon_state);
            if (hud_text_changed(s_last_recon_state, sizeof(s_last_recon_state), state_text)) {
                const uint16_t box_x0 = 18;
                const uint16_t box_y0 = 70;
                const uint16_t box_x1 = (uint16_t)(LCD_WIDTH - 19);
                const uint16_t box_y1 = 170;
                hud_fill(box_x0, box_y0, box_x1, box_y1, BLACK);
                uint16_t text_w = (uint16_t)(strlen(state_text) * LCD_CHAR_W(40));
                uint16_t text_x = 40;
                if (text_w < 240) {
                    text_x = (uint16_t)((LCD_WIDTH - text_w) / 2U);
                }
                lcd_show_string(text_x, 96, 240, 60, 40, (char *)state_text, GRAY);
                lcd_show_string((uint16_t)(text_x + 1), 96, 240, 60, 40, (char *)state_text, GRAY);
            }
            draw_recon_sequence();
        } else if (s_current_page == DISP_PAGE_CAMERA) {
            const uint16_t top_y = 10;
            const uint16_t top_h = 16;

            const char *x_text = "X:--.--";
            const char *y_text = "Y:--.--";
            const char *yaw_text = "A:--.-";
            const char *conf_text = "C:--";

            if (s_current_state.aruco_valid) {
                snprintf(str_buf, sizeof(str_buf), "X:%+05.2f", s_current_state.aruco_x);
                x_text = str_buf;
                if (hud_text_changed(s_last_cam_x, sizeof(s_last_cam_x), x_text)) {
                    hud_draw_text_region(12, top_y, 84, top_h, 16, x_text, WHITE);
                }

                snprintf(str_buf, sizeof(str_buf), "Y:%+05.2f", s_current_state.aruco_y);
                y_text = str_buf;
                if (hud_text_changed(s_last_cam_y, sizeof(s_last_cam_y), y_text)) {
                    hud_draw_text_region(106, top_y, 84, top_h, 16, y_text, WHITE);
                }

                snprintf(str_buf, sizeof(str_buf), "A:%+04.1f", s_current_state.aruco_yaw);
                yaw_text = str_buf;
                if (hud_text_changed(s_last_cam_yaw, sizeof(s_last_cam_yaw), yaw_text)) {
                    hud_draw_text_region(200, top_y, 72, top_h, 16, yaw_text, YELLOW);
                }

                snprintf(str_buf, sizeof(str_buf), "C:%0.2f", s_current_state.aruco_conf);
                conf_text = str_buf;
                if (hud_text_changed(s_last_cam_conf, sizeof(s_last_cam_conf), conf_text)) {
                    hud_draw_text_region(272, top_y, 44, top_h, 16, conf_text, UI_GREEN);
                }
            } else {
                if (hud_text_changed(s_last_cam_x, sizeof(s_last_cam_x), x_text)) {
                    hud_draw_text_region(12, top_y, 84, top_h, 16, x_text, GRAY);
                }
                if (hud_text_changed(s_last_cam_y, sizeof(s_last_cam_y), y_text)) {
                    hud_draw_text_region(106, top_y, 84, top_h, 16, y_text, GRAY);
                }
                if (hud_text_changed(s_last_cam_yaw, sizeof(s_last_cam_yaw), yaw_text)) {
                    hud_draw_text_region(200, top_y, 72, top_h, 16, yaw_text, GRAY);
                }
                if (hud_text_changed(s_last_cam_conf, sizeof(s_last_cam_conf), conf_text)) {
                    hud_draw_text_region(272, top_y, 44, top_h, 16, conf_text, GRAY);
                }
            }
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
    s_current_state.recon_state = DISP_RECON_IDLE;
    s_current_state.recon_order_count = 0;
    s_current_page = DISP_PAGE_STATUS;
    s_page_dirty = true;

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

void disp_mgr_update_aruco(float x, float y, float yaw, float conf, bool valid)
{
    s_current_state.aruco_x = x;
    s_current_state.aruco_y = y;
    s_current_state.aruco_yaw = yaw;
    s_current_state.aruco_conf = conf;
    s_current_state.aruco_valid = valid;
}

void disp_mgr_update_recon(const char *own_car_id,
                           const char order[][10],
                           int order_count,
                           disp_recon_state_t state)
{
    if (own_car_id && own_car_id[0] != '\0') {
        strncpy(s_current_state.car_id, own_car_id, sizeof(s_current_state.car_id) - 1);
        s_current_state.car_id[sizeof(s_current_state.car_id) - 1] = '\0';
    }

    s_current_state.recon_state = state;
    s_current_state.recon_order_count = 0;
    if (order && order_count > 0) {
        int count = order_count > 4 ? 4 : order_count;
        for (int i = 0; i < count; i++) {
            strncpy(s_current_state.recon_order[i], order[i], sizeof(s_current_state.recon_order[i]) - 1);
            s_current_state.recon_order[i][sizeof(s_current_state.recon_order[i]) - 1] = '\0';
        }
        s_current_state.recon_order_count = count;
    }
}

void disp_mgr_set_page(disp_page_t page)
{
    if (page != s_current_page) {
        s_current_page = page;
        if (s_current_page == DISP_PAGE_CAMERA) {
            lcd_set_invert(true);
            lcd_clear(BLACK);
        } else {
            lcd_set_invert(false);
        }
        s_page_dirty = true;
    }
}

disp_page_t disp_mgr_get_page(void)
{
    return s_current_page;
}

void disp_mgr_next_page(void)
{
    if (s_current_page == DISP_PAGE_STATUS) {
        disp_mgr_set_page(DISP_PAGE_RECON);
    } else if (s_current_page == DISP_PAGE_RECON) {
        disp_mgr_set_page(DISP_PAGE_CAMERA);
    } else {
        disp_mgr_set_page(DISP_PAGE_STATUS);
    }
}
