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
    DISP_PAGE_CAMERA
} disp_page_t;

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


#ifdef __cplusplus
}
#endif
