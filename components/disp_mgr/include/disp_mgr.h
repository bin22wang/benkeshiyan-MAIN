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
    DISP_PAGE_RECON,
    DISP_PAGE_CAMERA
} disp_page_t;

typedef enum {
    DISP_RECON_IDLE = 0,
    DISP_RECON_PREPARING,
    DISP_RECON_READY,
    DISP_RECON_ASSEMBLING,
    DISP_RECON_GUIDING,
    DISP_RECON_SUCCESS
} disp_recon_state_t;

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

void disp_mgr_update_aruco(float x, float y, float yaw, float conf, bool valid);

void disp_mgr_update_recon(const char *own_car_id,
                           const char order[][10],
                           int order_count,
                           disp_recon_state_t state);


#ifdef __cplusplus
}
#endif
