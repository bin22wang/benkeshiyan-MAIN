#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Formation control tuning (migrated from vel_solver.h). */
#define FORMATION_POS_TOLERANCE        0.15f
#define FORMATION_MIN_SPEED            0.05f
#define FORMATION_KP                   0.7f
#define FORMATION_KD                   0.0f
#define FORMATION_VEL_GAIN             1.05f
#define FORMATION_LEADER_DEADZONE      0.04f
#define FORMATION_MOTION_THRESHOLD     0.05f
#define FORMATION_STILL_COUNT          4
#define FORMATION_YAW_KP               0.05f

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
