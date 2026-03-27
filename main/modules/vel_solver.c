/*
 * vel_solver.c
 *
 * ESP32 侧速度解算模块（对齐 STM32 balance.c / formation_control.c 逻辑）。
 * 主要职责：
 *  - 基于上层指令（移动/编队）与本车状态计算目标速度
 *  - 输出 body-frame 速度: Vx(前向) / Vy(左向) / Vz(yaw控制量)
 *
 * 说明：
 *  - Vz 为归一化 yaw 控制量（范围 [-1, 1]），直接送 STM32 的 Drive_Motor()
 *  - 本模块不负责 UART 发送，仅输出 vel_output_t
 */

#include "vel_solver.h"
#include "proto_parser.h"
#include "formation_algo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "vel_solver";

/* 内部状态 */
static vel_solver_mode_t s_mode = VEL_MODE_IDLE;
static vel_own_state_t s_own = {0};
static vel_output_t s_output = {0};

/* 位置目标（世界坐标） */
static float s_target_x = 0.0f;
static float s_target_y = 0.0f;
static float s_target_yaw = 0.0f;

/* Follower 参数 */
static char s_leader_id[PROTO_CAR_ID_LEN] = {0};
static float s_off_x = 0.0f;
static float s_off_y = 0.0f;
static float s_off_yaw = 0.0f;

/* 手动遥控速度（VEL_MODE_MANUAL 模式下直接透传） */
static float s_manual_vx = 0.0f;
static float s_manual_vy = 0.0f;
static float s_manual_vz = 0.0f;

/* yaw PID */ 
static float    s_yaw_integral     = 0.0f;
static float    s_last_yaw_error   = 0.0f;
static float    s_last_pid_output  = 0.0f;
static uint32_t s_last_pid_state_ts = 0;

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void reset_control_state(void)
{
    s_yaw_integral      = 0.0f;
    s_last_yaw_error    = 0.0f;
    s_last_pid_output   = 0.0f;
    s_last_pid_state_ts = 0;
}


/**
 * Yaw PID (fixed: dead-band + state-gated integration)
 *
 * Fix 1 - integral oversampling:
 *   STM32 reports state at 10 Hz but uart_tx_task calls this at 50 Hz.
 *   Without gating, s_yaw_integral accumulates 5x per real state change,
 *   making effective KI 5x larger -> overshoot -> oscillation.
 *   Fix: only advance integral/derivative when s_own.timestamp_ms changes.
 *
 * Fix 2 - missing dead-band:
 *   Near target, sensor noise produces tiny non-zero errors, causing the
 *   PID to continuously output small corrections -> persistent chatter.
 *   Fix: return 0 and reset integral when |error| < VEL_SOLVER_YAW_DEADBAND.
 */
static float yaw_pid_control(float current_yaw, float target_yaw)
{
    float yaw_error = target_yaw - current_yaw;
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    /* Dead-band: suppress output near target to prevent sensor-noise chatter */
    if (fabsf(yaw_error) < VEL_SOLVER_YAW_DEADBAND) {
        s_yaw_integral      = 0.0f;
        s_last_yaw_error    = yaw_error;
        s_last_pid_output   = 0.0f;
        return 0.0f;
    }

    /* Return cached output when sensor state has not changed (50Hz call / 10Hz state) */
    if (s_own.timestamp_ms == s_last_pid_state_ts) {
        return s_last_pid_output;
    }

    float derivative = yaw_error - s_last_yaw_error;
    s_yaw_integral += yaw_error;

    s_yaw_integral = clamp_float(s_yaw_integral,
                                 -VEL_SOLVER_YAW_INTEGRAL_MAX,
                                 VEL_SOLVER_YAW_INTEGRAL_MAX);

    float yaw_output = VEL_SOLVER_YAW_KP * yaw_error
                     + VEL_SOLVER_YAW_KI * s_yaw_integral
                     + VEL_SOLVER_YAW_KD * derivative;

    s_last_yaw_error    = yaw_error;
    s_last_pid_state_ts = s_own.timestamp_ms;
    s_last_pid_output   = clamp_float(yaw_output,
                                      -VEL_SOLVER_YAW_OUTPUT_MAX,
                                      VEL_SOLVER_YAW_OUTPUT_MAX);
    return s_last_pid_output;
}

void vel_solver_init(void)
{
    memset(&s_own, 0, sizeof(s_own));
    memset(&s_output, 0, sizeof(s_output));
    s_mode = VEL_MODE_IDLE;
    reset_control_state();
}

void vel_solver_update_own_state(float x, float y, float yaw, float vx, float vy, float vz)
{
    s_own.x = x;
    s_own.y = y;
    s_own.yaw = yaw;
    s_own.vx = vx;
    s_own.vy = vy;
    s_own.vz = vz;
    s_own.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    s_own.valid = true;
}

const vel_own_state_t *vel_solver_get_own_state(void)
{
    return &s_own;
}

void vel_solver_set_position_target(float target_x, float target_y, float target_yaw)
{
    s_target_x = target_x;
    s_target_y = target_y;
    s_target_yaw = target_yaw;
    s_mode = VEL_MODE_POSITION;
    reset_control_state();
    ESP_LOGI(TAG, "set position target (%.3f, %.3f, %.1f)", target_x, target_y, target_yaw);
}

void vel_solver_set_leader_mode(void)
{
    /* Enter leader mode: hold current pose until a new move command arrives. */
    if (s_own.valid) {
        s_target_x = s_own.x;
        s_target_y = s_own.y;
        s_target_yaw = s_own.yaw;
    } else {
        s_target_x = 0.0f;
        s_target_y = 0.0f;
        s_target_yaw = 0.0f;
    }
    s_mode = VEL_MODE_POSITION;
    reset_control_state();
}

void vel_solver_set_follower_mode(const char *leader_id, float offset_x, float offset_y, float offset_yaw)
{
    strncpy(s_leader_id, leader_id, PROTO_CAR_ID_LEN - 1);
    s_leader_id[PROTO_CAR_ID_LEN - 1] = '\0';
    s_off_x = offset_x;
    s_off_y = offset_y;
    s_off_yaw = offset_yaw;
    s_mode = VEL_MODE_FOLLOWER;
    reset_control_state();
    ESP_LOGI(TAG, "set follower leader=%s off=(%.2f, %.2f, %.1f)", s_leader_id, s_off_x, s_off_y, s_off_yaw);
}

void vel_solver_update_follower_offset(float offset_x, float offset_y, float offset_yaw)
{
    s_off_x = offset_x;
    s_off_y = offset_y;
    s_off_yaw = offset_yaw;
    ESP_LOGI(TAG, "update follower offset=(%.2f, %.2f, %.1f)", offset_x, offset_y, offset_yaw);
}

void vel_solver_set_manual_speed(float target_vx, float target_vy, float target_vz)
{
    s_manual_vx = target_vx;
    s_manual_vy = target_vy;
    s_manual_vz = target_vz;
    s_mode = VEL_MODE_MANUAL;
    reset_control_state();
    ESP_LOGI(TAG, "set manual speed (%.3f, %.3f, %.3f)", target_vx, target_vy, target_vz);
}

void vel_solver_stop(void)
{
    s_mode = VEL_MODE_IDLE;
    s_output.target_vx = 0.0f;
    s_output.target_vy = 0.0f;
    s_output.target_vz = 0.0f;
    s_output.valid = false;
    reset_control_state();
    ESP_LOGI(TAG, "vel_solver stop -> IDLE");
}

vel_solver_mode_t vel_solver_get_mode(void)
{
    return s_mode;
}

void vel_solver_notify_car_updated(const char *car_id)
{
    (void)car_id;
}

static bool own_state_ready(void)
{
    if (!s_own.valid) {
        ESP_LOGW(TAG, "own state invalid, skip compute");
        return false;
    }
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - s_own.timestamp_ms > 2000) {
        ESP_LOGW(TAG, "own state stale (>%u ms), skip", (unsigned int)(now - s_own.timestamp_ms));
        return false;
    }
    return true;
}

bool vel_solver_compute(vel_output_t *out)
{
    if (!out) return false;
    memset(out, 0, sizeof(*out));

    if (!own_state_ready()) return false;

    float control_vx = 0.0f;
    float control_vy = 0.0f;
    float control_vz = 0.0f;

    if (s_mode == VEL_MODE_IDLE) {
        s_output.valid = false;
        *out = s_output;
        return false;
    }

    if (s_mode == VEL_MODE_MANUAL) {
        control_vx = s_manual_vx;
        control_vy = s_manual_vy;
        control_vz = s_manual_vz;
    } else if (s_mode == VEL_MODE_POSITION) {
        /* Auto_Adjust_Position_And_Yaw 对齐实现 */
        float error_x = s_target_x - s_own.x;
        float error_y = s_target_y - s_own.y;
        float distance_to_target = sqrtf(error_x * error_x + error_y * error_y);

        if (distance_to_target < VEL_SOLVER_POS_TOLERANCE) {
            control_vx = 0.0f;
            control_vy = 0.0f;
            control_vz = yaw_pid_control(s_own.yaw, s_target_yaw);
        } else {
            float global_target_angle = atan2f(error_y, error_x) * 180.0f / (float)M_PI;
            float relative_target_angle = global_target_angle - s_own.yaw;
            while (relative_target_angle > 180.0f) relative_target_angle -= 360.0f;
            while (relative_target_angle < -180.0f) relative_target_angle += 360.0f;

            float base_speed = VEL_SOLVER_POS_KP * distance_to_target;
            if (base_speed > VEL_SOLVER_MAX_SPEED) {
                base_speed = VEL_SOLVER_MAX_SPEED;
            }
            if (distance_to_target < VEL_SOLVER_SLOWDOWN_RADIUS) {
                base_speed *= (distance_to_target / VEL_SOLVER_SLOWDOWN_RADIUS);
            }

            control_vx = base_speed * cosf(relative_target_angle * (float)M_PI / 180.0f);
            control_vy = base_speed * sinf(relative_target_angle * (float)M_PI / 180.0f);
            control_vz = yaw_pid_control(s_own.yaw, s_target_yaw);
        }
    } else if (s_mode == VEL_MODE_FOLLOWER) {  /* NOLINT: intentional else-if chain */
        /* Formation_Follower_Control 对齐实现 */
        const proto_car_state_t *leader = proto_get_car_state(s_leader_id);
        if (!leader || !leader->valid) {
            ESP_LOGW(TAG, "leader %s not found/valid", s_leader_id);
            reset_control_state();
            return false;
        }

        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (now - leader->last_update_ms > 2000) {
            reset_control_state();
            return false;
        }
        formation_out_t follower_out = {0};
        formation_calc_follower_vel(leader->x,
                                    leader->y,
                                    leader->yaw,
                                    leader->vx,
                                    leader->vy,
                                    s_own.x,
                                    s_own.y,
                                    s_own.yaw,
                                    s_off_x,
                                    s_off_y,
                                    s_off_yaw,
                                    FORMATION_KP,
                                    FORMATION_KD,
                                    &follower_out);

        control_vx = follower_out.target_vx;
        control_vy = follower_out.target_vy;
        control_vz = follower_out.target_vz;

        float control_speed = sqrtf(control_vx * control_vx + control_vy * control_vy);
        if (control_speed > VEL_SOLVER_MAX_SPEED) {
            control_vx = control_vx * VEL_SOLVER_MAX_SPEED / control_speed;
            control_vy = control_vy * VEL_SOLVER_MAX_SPEED / control_speed;
        }
    }

    s_output.target_vx = control_vx;
    s_output.target_vy = control_vy;
    s_output.target_vz = control_vz;
    s_output.valid = true;
    *out = s_output;
    return true;
}


