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

void disp_mgr_init(void);

/**
 * @brief Update telemetry data.
 * Calling this function refreshes the internal timestamp, keeping the UPLINK status "SYNCED".
 */
void disp_mgr_update_status(const char *car_id, bool wifi_connected,
                            sys_mode_t mode, float x, float y, float yaw);

#ifdef __cplusplus
}
#endif
