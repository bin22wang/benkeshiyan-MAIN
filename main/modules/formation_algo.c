#include "formation_algo.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float wrap_deg(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

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
                                 formation_out_t *out)
{
    if (!out) return;

    float L_yaw = L_yaw_deg * (float)M_PI / 180.0f;
    float F_yaw = F_yaw_deg * (float)M_PI / 180.0f;

    /* Step 1: target pose in global frame. */
    float T_x = L_x + offset_x * cosf(L_yaw) - offset_y * sinf(L_yaw);
    float T_y = L_y + offset_x * sinf(L_yaw) + offset_y * cosf(L_yaw);
    float T_yaw = wrap_deg(L_yaw_deg + offset_yaw_deg);

    /* Step 2: position feedback in global frame (with Filtered PD control) */
    float raw_err_x = T_x - F_x;
    float raw_err_y = T_y - F_y;

    /* Keep last error and filtered derivative. */
    static float s_last_err_x = 0.0f;
    static float s_last_err_y = 0.0f;
    static float s_filtered_d_err_x = 0.0f;
    static float s_filtered_d_err_y = 0.0f;

    float raw_d_err_x = raw_err_x - s_last_err_x;
    float raw_d_err_y = raw_err_y - s_last_err_y;

    s_last_err_x = raw_err_x;
    s_last_err_y = raw_err_y;

    float alpha = 0.15f;
    s_filtered_d_err_x = alpha * raw_d_err_x + (1.0f - alpha) * s_filtered_d_err_x;
    s_filtered_d_err_y = alpha * raw_d_err_y + (1.0f - alpha) * s_filtered_d_err_y;

    float err_x = raw_err_x;
    float err_y = raw_err_y;
    float err_dist = sqrtf(err_x * err_x + err_y * err_y);

    if (err_dist < FORMATION_POS_TOLERANCE) {
        err_x = 0.0f;
        err_y = 0.0f;
    }

    float V_fb_x = kp_pos * err_x + kd_pos * s_filtered_d_err_x;
    float V_fb_y = kp_pos * err_y + kd_pos * s_filtered_d_err_y;

    /* Step 3: leader velocity feedforward in global frame. */
    float V_ff_x = L_vx * cosf(L_yaw) - L_vy * sinf(L_yaw);
    float V_ff_y = L_vx * sinf(L_yaw) + L_vy * cosf(L_yaw);

    /* Step 4: compose and project to follower body frame. */
    float V_global_x = V_ff_x + V_fb_x;
    float V_global_y = V_ff_y + V_fb_y;
    out->target_vx = V_global_x * cosf(F_yaw) + V_global_y * sinf(F_yaw);
    out->target_vy = -V_global_x * sinf(F_yaw) + V_global_y * cosf(F_yaw);

    /* Step 5: yaw control. */
    float err_yaw = wrap_deg(T_yaw - F_yaw_deg);
    out->target_vz = err_yaw * FORMATION_YAW_KP;
}
