#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Formation control tuning (migrated from vel_solver.h). */
#define FORMATION_POS_TOLERANCE        0.15f  // 位置误差阈值（米），低于此值视为到达目标
#define FORMATION_MIN_SPEED            0.05f  // 最小移动速度（米/秒），低于此值视为静止
#define FORMATION_KP                   0.7f   // 位置环比例增益
#define FORMATION_KD                   0.0f   // 位置环微分增益
#define FORMATION_VEL_GAIN             1.05f  // 速度前馈增益（>1 可适当补偿系统延迟）
#define FORMATION_LEADER_DEADZONE      0.04f  // 领航者速度死区，低于此值视为静止
#define FORMATION_MOTION_THRESHOLD     0.05f  // 速度合成后的小车运动阈值，低于此值视为静止
#define FORMATION_STILL_COUNT          4      // 连续多少帧（50Hz）低于运动阈值则认为编队静止，重置 PID 状态
#define FORMATION_YAW_KP               0.05f  // 航向误差比例增益（直接乘以误差得到角速度指令）

/* 1: 全局坐标系偏移, 0: 领航者局部坐标系偏移 */
#define FORMATION_OFFSET_MODE_GLOBAL   1

typedef struct {
    float target_vx;
    float target_vy;
    float target_vz;
} formation_out_t;

void formation_calc_follower_vel(float L_x,
                                 float L_y,
                                 float L_yaw_deg,
                                 float L_vx,
                                 float L_vy,
                                 float F_x,
                                 float F_y,
                                 float F_yaw_deg,
                                 float offset_x,
                                 float offset_y,
                                 float offset_yaw_deg,
                                 float kp_pos,
                                 float kd_pos,
                                 formation_out_t *out);

#ifdef __cplusplus
}
#endif
