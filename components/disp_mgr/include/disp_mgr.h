/**
 * @file disp_mgr.h
 * @brief Cyber-Mecha HUD Display Manager (320x240 Landscape)
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Tactical Mode Enumeration
typedef enum {
    SYS_MODE_IDLE = 0,    // Displayed as STANDBY
    SYS_MODE_LEADER,      // Displayed as VANGUARD
    SYS_MODE_FOLLOW,      // Displayed as WINGMAN
    SYS_MODE_MANUAL       // Displayed as OVERRIDE
} sys_mode_t;

typedef enum {
    DISP_PAGE_STATUS = 0,
    DISP_PAGE_TOF,
    DISP_PAGE_CALIB
} disp_page_t;

typedef struct {
    uint8_t focus;
    uint8_t done_mask;
    bool using_defaults;
    float left_scale;
    float left_bias;
    float right_scale;
    float right_bias;
} disp_calib_state_t;

void disp_mgr_init(void);

void disp_mgr_set_page(disp_page_t page);
void disp_mgr_next_page(void);
disp_page_t disp_mgr_get_page(void);

/**
 * @brief Update telemetry data.
 * Calling this function refreshes the internal timestamp, keeping the UPLINK status "SYNCED".
 */
void disp_mgr_update_status(const char *car_id, bool wifi_connected,
                            sys_mode_t mode, float x, float y, float yaw);

void disp_mgr_update_tof(uint16_t left_mm, bool left_ok,
                         uint16_t right_mm, bool right_ok);

void disp_mgr_update_calib(const disp_calib_state_t *state);

#ifdef __cplusplus
}
#endif
