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
    SYS_MODE_MANUAL,      // Displayed as OVERRIDE
    SYS_MODE_AUTO         // Displayed as AUTO
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

/**
 * @brief 更新重构诊断信息（用于 RECON 页地面测试排障）。
 *
 * 该信息展示在第二页（RECON），用于在无串口条件下现场定位问题：
 *  - rx_total     : 本车累计收到的“下行指令”总条数（凡是进入 recon 解析的报文都计数），
 *                   用来判断“服务器→小车下行链路是否真的把包送达本车”。
 *  - rx_self      : 其中“目标是本车”的指令条数（STEP/GUIDE 命中自己），
 *                   用来判断“本车是否真的被点名”。
 *  - last_cmd     : 最近一条与本车相关的下行指令短标签，如 "STEP" "GUIDE" "ASM" "PREP"。
 *  - vx,vy,vz     : 最近一次收到的制导速度值（m/s、rad/s），用来核对制导是否真的下发到本车。
 *  - guide_fresh  : 最近的制导指令是否“新鲜”（如 1s 内收到过），false 时速度显示为灰色。
 *
 * 线程安全：仅做标量赋值，由 recon/proto 线程调用、disp 线程读取，允许极短暂竞态（仅显示用途）。
 */
void disp_mgr_update_recon_rx(uint32_t rx_total, uint32_t rx_self,
                              const char *last_cmd);

// 最近一次制导速度（车体系，m/s 与 rad/s），fresh=true 表示刚收到（1s 内），
// 用于在 RECON 页核对制导是否真的下发到本车、以及数值大小是否合理。
void disp_mgr_update_recon_guide(float vx, float vy, float vz, bool fresh);


#ifdef __cplusplus
}
#endif
