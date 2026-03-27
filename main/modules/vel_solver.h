/**
 * @file vel_solver.h
 * @brief 速度解算模块（对应 STM32 端的 formation_control.c 中的控制逻辑）
 *
 * ====== 功能说明 ======
 * 本模块在 ESP32 侧完成原本 STM32 负责的速度解算，将解析到的目标指令
 * 换算为小车期望速度 [target_Vx, target_Vy, target_Vz]，
 * 供后续 UART 桥接模块直接下发给 STM32 做电机控制。
 *
 * ====== 控制模式 ======
 *
 *  模式 0：无控制（IDLE）
 *    └─ 输出 [0, 0, 0]
 *
 *  模式 1：位姿闭环（对应 [C,...] 移动指令 / Formation Leader）
 *    └─ 输入：本车当前位姿 (x, y, yaw) + 目标位姿 (tx, ty, tyaw)
 *    └─ 算法：全局坐标系 PD → 转换到本体坐标系 → 输出 [Vx, Vy, Vz]
 *    └─ 对应 STM32: Auto_Adjust_Position_And_Yaw()
 *
 *  模式 2：编队 Follower 控制（对应 [F,F,...] 编队跟随指令）
 *    └─ 输入：本车当前位姿 + Leader 实时位姿/速度 + 偏移量 (ox, oy, oyaw)
 *    └─ 算法：PD 位置控制 + 速度前馈混合（自适应切换）
 *    └─ 对应 STM32: Formation_Follower_Control()
 *
 * ====== 依赖说明 ======
 * 本模块需要本车实时位姿作为反馈量（x, y, yaw, vx, vy, vz）。
 * 该数据来源：STM32 → ESP32 UART（uart_bridge 模块桥接后填入）。
 * 在 uart_bridge 未就绪前，可调用 vel_solver_update_own_state() 手动填入。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 常量与调参宏（与 STM32 balance.c / formation_control.c 保持一致）
 *
 * 对应关系（STM32变量名 -> 本宏名）：
 *   position_tolerance   -> VEL_SOLVER_POS_TOLERANCE
 *   max_linear_speed     -> VEL_SOLVER_MAX_SPEED
 *   Pos_KP               -> VEL_SOLVER_POS_KP
 *   Yaw_KP/KI/KD         -> VEL_SOLVER_YAW_KP/KI/KD
 * ======================================================================*/

/**
 * 位置到达容差（米）
 * STM32: position_tolerance = 0.05f (balance.c)
 * 注意：编队 follower 模式用的是 Formation_tolerance = 0.15f
 */
#define VEL_SOLVER_POS_TOLERANCE    0.05f


/**
 * 最大线速度输出（m/s），直接送入 STM32 Drive_Motor(Vx,Vy,Vz)
 * STM32: max_linear_speed = 0.3f / Formation_max_speed = 0.3f
 * 为 Follower 预留额外线速度裕量，用于追赶位置误差
 */
#define VEL_SOLVER_MAX_SPEED        0.45f


/**
 * 独立移动位置比例系数（对应 Auto_Adjust_Position_And_Yaw）
 * STM32: Pos_KP = 0.5f
 */
#define VEL_SOLVER_POS_KP           0.5f

/* Formation tuning moved to formation_algo.h. */

/**
 * Yaw PID 比例系数
 * STM32: Yaw_KP = 0.12f
 * 输出单位：[-1, 1]，直接作为 Drive_Motor 的 Vz 参数
 */
#define VEL_SOLVER_YAW_KP           0.05f

/**
 * Yaw PID 积分系数
 * STM32: Yaw_KI = 0.001f
 */
#define VEL_SOLVER_YAW_KI           0.00f

/**
 * Yaw PID 微分系数
 * STM32: Yaw_KD = 0.05f
 */
#define VEL_SOLVER_YAW_KD           0.02f

/**
 * Yaw PID 积分限幅 = 1.0 / Yaw_KI（与 STM32 动态计算逻辑一致）
 * 约束积分项单独贡献最多 ±1.0 的输出量
 */
#define VEL_SOLVER_YAW_INTEGRAL_MAX (1.0f / VEL_SOLVER_YAW_KI)

/**
 * Yaw PID 输出限幅（与 STM32 target_limit_float 一致）
 */
#define VEL_SOLVER_YAW_OUTPUT_MAX   1.0f

/**
 * Yaw PID 死区（度）：误差绝对值小于此值时直接输出 0 并清零积分，
 * 防止接近目标角度时传感器噪声引起持续震荡。
 * 建议值：1.5~3.0 度（与实际传感器噪声水平匹配）。
 */
#define VEL_SOLVER_YAW_DEADBAND     0.8f

/**
 * 近距离减速半径（米）：distance < 此值时开始平滑减速
 * STM32: if(distance_to_target < 0.5f) 和 if(distance < 0.5f)
 */
#define VEL_SOLVER_SLOWDOWN_RADIUS  0.5f

/* ========================================================================
 * 数据类型
 * ======================================================================*/

/**
 * @brief 控制模式枚举
 */
typedef enum {
    VEL_MODE_IDLE       = 0, /**< 无控制，输出全零                         */
    VEL_MODE_POSITION   = 1, /**< 位姿闭环（移动指令 / Leader 自主导航）   */
    VEL_MODE_FOLLOWER   = 2, /**< 编队 Follower 控制                       */
    VEL_MODE_MANUAL     = 3, /**< 手动遥控（前端键盘直接下发速度透传）      */
} vel_solver_mode_t;

/**
 * @brief 本车实时状态（由 STM32 通过 UART 上报，uart_bridge 负责填入）
 *
 * 坐标系说明（与 STM32 balance.c 保持一致）：
 *   - x, y    全局坐标（米），由里程计/视觉定位提供
 *   - yaw     全局航向角（度），STM32 上报原始值直接使用
 *   - vx      车体坐标系，车头方向为正（m/s），由 Calculate_Car_Velocity() 计算
 *   - vy      车体坐标系，车头左方为正（m/s）
 *   - vz      车体坐标系，绕 Z 轴逆时针为正（rad/s，由 Calculate_Car_Velocity() 计算）
 *
 * 注意：vx/vy/vz 均为本车 body-frame，与 Drive_Motor(Vx,Vy,Vz) 的输入约定相同。
 */
typedef struct {
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float vz;
    uint32_t timestamp_ms; /**< 最后更新时间（ms），用于超时检测 */
    bool  valid;           /**< false = STM32 尚未上报，不可用于解算 */
} vel_own_state_t;

/**
 * @brief 速度解算输出（直接发给 STM32 的 Drive_Motor(Vx, Vy, Vz)）
 *
 * 坐标系：**本体坐标系（body-frame），与 STM32 Drive_Motor 入参约定相同**
 *   - target_vx  车头朝前为正（m/s）
 *   - target_vy  车头朝左为正（m/s）
 *   - target_vz  yaw 控制量，范围 [-1.0, 1.0]（无量纲归一化）
 *                由 Yaw_PID_Control 输出，STM32 经 Smooth_control() 后
 *                直接参与麦克纳姆轮逆运动学计算
 */
typedef struct {
    float target_vx;
    float target_vy;
    float target_vz;
    bool  valid; /**< false = 当前无有效指令，保持 STM32 原有状态 */
} vel_output_t;

/* ========================================================================
 * 初始化 / 状态更新 API
 * ======================================================================*/

/**
 * @brief 初始化速度解算模块，清空所有状态。
 *        在 proto_parser_init() 之后调用一次。
 */
void vel_solver_init(void);

/**
 * @brief 更新本车实时位姿（由 uart_bridge 收到 STM32 上报后调用）。
 *
 * 这是解算的核心输入。如果长时间未调用（超时），解算器会输出零速度。
 *
 * @param x,y    全局位置（米）
 * @param yaw    全局航向角（度）
 * @param vx,vy  全局速度（米/秒）
 * @param vz     角速度
 */
void vel_solver_update_own_state(float x, float y, float yaw,
                                  float vx, float vy, float vz);

/**
 * @brief 获取当前本车状态（只读）。
 */
const vel_own_state_t *vel_solver_get_own_state(void);

/* ========================================================================
 * 指令设置 API（由 proto_parser 内部调用）
 * ======================================================================*/

/**
 * @brief 设置位姿目标（对应 [C,CARn,tx,ty,tyaw] 移动指令）。
 *
 * 调用后进入 VEL_MODE_POSITION 模式，vel_solver_compute() 开始输出速度。
 *
 * @param target_x    目标 X 坐标（米）
 * @param target_y    目标 Y 坐标（米）
 * @param target_yaw  目标航向角（度）
 */
void vel_solver_set_position_target(float target_x, float target_y, float target_yaw);

/**
 * @brief 设置编队 Leader 角色（对应 [F,L,...] 指令）。
 *
 * Leader 的运动目标由上位机通过 [C,...] 指令设置，
 * 因此实际上复用 VEL_MODE_POSITION 模式。
 */
void vel_solver_set_leader_mode(void);

/**
 * @brief 设置编队 Follower 参数（对应 [F,F,leader,ox,oy,oyaw] 指令）。
 *
 * 调用后进入 VEL_MODE_FOLLOWER 模式。
 * 每次广播刷新 Leader 状态后（proto_parse_message 调用 update_car_state），
 * vel_solver_compute() 会自动使用最新的 Leader 位姿。
 *
 * @param leader_id   Leader 的车辆 ID 字符串，如 "CAR1"
 * @param offset_x    在 Leader 本体坐标系下的 X 偏移（米）
 * @param offset_y    在 Leader 本体坐标系下的 Y 偏移（米）
 * @param offset_yaw  航向角偏移（度）
 */
void vel_solver_set_follower_mode(const char *leader_id,
                                   float offset_x, float offset_y, float offset_yaw);

/**
 * @brief 更新编队 Follower 偏移量（对应 [F,U,...] 指令）。
 *        仅在 Follower 模式下有效。
 */
void vel_solver_update_follower_offset(float offset_x, float offset_y, float offset_yaw);

/**
 * @brief 停止所有控制，进入 IDLE 模式（对应 [F,T] 指令）。
 *        输出全零速度。
 */
void vel_solver_stop(void);

/**
 * @brief 设置手动遥控速度（对应 [M,CARn,vx,vy,vz] 指令）。
 *
 * 调用后进入 VEL_MODE_MANUAL 模式，vel_solver_compute() 直接透传本次速度，
 * 不经过任何 PID 闭环解算。适用于前端键盘实时遥控场景。
 *
 * @param target_vx  期望线速度 X（m/s，车头朝前为正）
 * @param target_vy  期望线速度 Y（m/s，车头朝左为正）
 * @param target_vz  期望角速度 Z（归一化 [-1,1]）
 */
void vel_solver_set_manual_speed(float target_vx, float target_vy, float target_vz);

/* ========================================================================
 * 解算入口 API
 * ======================================================================*/

/**
 * @brief 执行一次速度解算，返回期望速度输出。
 *
 * 调用时机：在 proto_parse_message() 处理完所有消息后调用，
 * 或在独立的定时任务中周期调用（建议 20~50ms 一次，与广播频率一致）。
 *
 * 内部会根据当前模式选择控制算法：
 *   - IDLE      → 输出 [0, 0, 0]
 *   - POSITION  → PD 位姿闭环
 *   - FOLLOWER  → 自适应 PD + 速度前馈
 *
 * @param[out] out  解算结果，调用者负责传入有效指针
 * @return    true  有效输出（应将 out 发给 STM32）
 *            false 无有效指令 / 本车状态超时（不应发送）
 */
bool vel_solver_compute(vel_output_t *out);

/**
 * @brief 获取当前控制模式（调试用）。
 */
vel_solver_mode_t vel_solver_get_mode(void);

/* ========================================================================
 * Leader 状态注入（由 proto_parser 内部调用）
 * ======================================================================*/

/**
 * @brief 通知解算器某辆远端车辆的状态已更新。
 *
 * proto_parser 在 parse_broadcast_cars() 成功更新 s_cars[] 后调用此函数，
 * 解算器内部直接读取 proto_get_car_state() 以获取最新 Leader 状态。
 * （此函数目前仅作为"数据就绪"信号，实际读取在 vel_solver_compute() 内完成）
 *
 * @param car_id 刚更新的车辆 ID
 */
void vel_solver_notify_car_updated(const char *car_id);

#ifdef __cplusplus
}
#endif
