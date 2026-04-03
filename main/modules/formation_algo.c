#include "formation_algo.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 将角度值限制在 [-180, 180) 范围内
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
    
    // 弧度制转换
    float L_yaw = L_yaw_deg * (float)M_PI / 180.0f;
    float F_yaw = F_yaw_deg * (float)M_PI / 180.0f;

    // 从领航者坐标系下的目标位置映射到全局坐标系的位置
    /* Step 1: target pose in global frame. */
    float T_x = L_x + offset_x * cosf(L_yaw) - offset_y * sinf(L_yaw);
    float T_y = L_y + offset_x * sinf(L_yaw) + offset_y * cosf(L_yaw);
    float T_yaw = wrap_deg(L_yaw_deg + offset_yaw_deg);
    
    // 计算映射后的全局坐标系下的位置误差，并进行 PD 控制，得到速度指令
    /* Step 2: position feedback in global frame (with Filtered PD control) */
    float raw_err_x = T_x - F_x;
    float raw_err_y = T_y - F_y;

    
    /* Keep last error and filtered derivative. */
    static float s_last_err_x = 0.0f;
    static float s_last_err_y = 0.0f;
    static float s_filtered_d_err_x = 0.0f;
    static float s_filtered_d_err_y = 0.0f;

    /* 2. 算误差的变化率（求导），并加低通滤波 */
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

        s_filtered_d_err_x = 0.0f; 
        s_filtered_d_err_y = 0.0f;
    }

    // 位置环反馈控制 （速度输出）
    float V_fb_x = kp_pos * err_x + kd_pos * s_filtered_d_err_x;
    float V_fb_y = kp_pos * err_y + kd_pos * s_filtered_d_err_y;

    // 前馈控制
    /* Step 3: leader velocity feedforward in global frame. */

    // 领航者速度死区处理，低于一定阈值视为静止，避免小幅度速度引起的抖动
    if (fabsf(L_vx) < FORMATION_LEADER_DEADZONE) { // 这个宏在头文件里是 0.04f
        L_vx = 0.0f;
    }
    if (fabsf(L_vy) < FORMATION_LEADER_DEADZONE) {
        L_vy = 0.0f;
    }
    
    // 将领航者的速度从其坐标系转换到全局坐标系，得到前馈速度
    float V_ff_x = L_vx * cosf(L_yaw) - L_vy * sinf(L_yaw);
    float V_ff_y = L_vx * sinf(L_yaw) + L_vy * cosf(L_yaw);

    /* 1. 速度合并 (世界坐标系下) */
    /* Step 4: compose and project to follower body frame. */
    // 将前馈和反馈速度叠加，并从全局坐标系转换到跟随者的坐标系，得到最终的速度指令
    float V_global_x = V_ff_x + V_fb_x;
    float V_global_y = V_ff_y + V_fb_y;
    out->target_vx = V_global_x * cosf(F_yaw) + V_global_y * sinf(F_yaw);
    out->target_vy = -V_global_x * sinf(F_yaw) + V_global_y * cosf(F_yaw);

    /* Step 5: yaw control. */
    // 航向控制只使用比例项，直接根据领航者与跟随者的航向误差计算角速度指令
    // 误差小于一定阈值时不控制旋转，避免小幅度误差引起的抖动
    float err_yaw = wrap_deg(T_yaw - F_yaw_deg);
    if (fabsf(err_yaw) < 0.8f) { // 0.8度以内不控制旋转，避免小幅度误差引起的抖动
        err_yaw = 0.0f;
    }
    out->target_vz = err_yaw * FORMATION_YAW_KP;
}
