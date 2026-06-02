/**
 * @file proto_parser.c
 * @brief UDP 协议解析器实现
 *
 * 负责识别并解析上位机（服务器/其他小车）通过 UDP 下发的所有消息格式，
 * 将解析结果保存在内部静态状态变量中，供上层通过 accessor API 消费。
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │  消息类型速查表                                                      │
 * ├──────┬──────────────────────────────┬──────────────────────────────┤
 * │  #   │  格式示例                    │  说明                        │
 * ├──────┼──────────────────────────────┼──────────────────────────────┤
 * │  1   │ [T,E,1]                      │ 拓扑使能/禁用                │
 * │  2   │ [T,M,0,1,1,1, 1,0,1,1, ...]  │ 4×4 拓扑矩阵                │
 * │  3   │ [C,CAR1,1.5,2.3,45.0]        │ 点对点移动指令               │
 * │  4   │ [F,T]                        │ 编队停止                     │
 * │  5   │ [F,S,CAR1,leader]            │ 编队启动                     │
 * │  6   │ [F,L,CAR1]                   │ 设置为 Leader                │
 * │  7   │ [F,F,CAR1,0.5,0.0,0.0]       │ 设置为 Follower + 偏移量     │
 * │  8   │ [F,U,CAR1,0.3,0.2,0.0]       │ 更新跟随偏移量               │
 * │  9   │ [3 C1 x y yaw vx vy vz ...]  │ 广播全场车辆状态             │
 * │  10  │ [T,CAR1,vx,vy,yaw]           │ 轨迹速度指令                  │
 * │  11  │ TOPOLOGY:0,1;1,0;...         │ 旧版拓扑矩阵（兼容）          │
 * │  12  │ TOPOLOGY_TOGGLE:True         │ 旧版拓扑使能切换（兼容）      │
 * └──────┴──────────────────────────────┴──────────────────────────────┘
 *
 * ====== 后续开发说明 ======
 * 每个子解析器函数末尾都有 "TODO: UART forward" 注释块，
 * 后期在那里调用 uart_bridge_send_xxx() 即可完成 ESP32→STM32 转发。
 */

#include "proto_parser.h"
#include "vel_solver.h"
#include "recon_mgr.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "proto";

/* ========================================================================
 * 内部状态变量
 * ========================================================================
 * 所有解析结果存储在以下静态变量中，模块外部通过 accessor API 只读访问。
 * 设计原则：每次解析覆盖旧值，消费者（app_main）调用 consume_xxx() 后
 * valid 标志自动清除，防止同一条消息被重复处理。
 * ======================================================================*/

/** 本车 ID，由 proto_parser_init() 写入，后续解析过滤时使用 */
static char s_own_id[PROTO_CAR_ID_LEN] = "CAR1";

/** 当前拓扑配置（矩阵 + 使能标志） */
static proto_topology_t      s_topology;

/** 最新收到的移动指令（仅保留发给本车的） */
static proto_cmd_move_t      s_move_cmd;

/** 最新收到的编队指令 */
static proto_cmd_formation_t s_formation_cmd;

/** 远端小车状态表，最多 PROTO_MAX_CARS 辆 */
static proto_car_state_t     s_cars[PROTO_MAX_CARS];


/* ========================================================================
 * 工具函数
 * ======================================================================*/

/**
 * @brief 将车辆 ID 字符串转换为 0-based 索引。
 *
 * 规则：CAR1→0, CAR2→1, CAR3→2, CAR4→3，未知→-1。
 * 索引与拓扑矩阵行/列下标对应。
 */
int proto_car_id_to_index(const char *car_id)
{
    if (strcmp(car_id, "CAR1") == 0) return 0;
    if (strcmp(car_id, "CAR2") == 0) return 1;
    if (strcmp(car_id, "CAR3") == 0) return 2;
    if (strcmp(car_id, "CAR4") == 0) return 3;
    return -1;
}

/**
 * @brief 原地去除字符串末尾的 \r、\n 和空格。
 *
 * 部分上位机在消息末尾附带换行符，统一清除以避免解析出错。
 */
static void strip_crlf(char *s)
{
    int len = (int)strlen(s);
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' '))
        s[--len] = '\0';
}

/**
 * @brief 返回当前系统时间（毫秒）。
 *
 * 使用 FreeRTOS Tick 计数换算，用于记录远端车辆状态的最后更新时间戳。
 */
static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static bool recon_filter_controls(void)
{
    recon_state_t state = recon_mgr_get_state();
    return state != RECON_STATE_IDLE;
}

/* -----------------------------------------------------------------------
 * 消息类型 10：轨迹速度指令
 * 格式：[T,CARn,vx,vy,target_yaw]
 *   CARn       - 目标车辆 ID（只处理发给本车的）
 *   vx,vy      - 线速度（m/s）
 *   target_yaw - 目标航向角（度）
 * ---------------------------------------------------------------------- */
static void parse_trajectory_cmd(const char *data)
{
    if (recon_filter_controls()) {
        ESP_LOGW(TAG, "[TRAJ] ignored during recon: %s", data);
        return;
    }

    char  target[PROTO_CAR_ID_LEN];
    float vx, vy, yaw;

    int n = sscanf(data, "[T,%9[^,],%f,%f,%f]", target, &vx, &vy, &yaw);
    if (n != 4) {
        ESP_LOGW(TAG, "[TRAJ] parse failed: %s", data);
        return;
    }

    if (strcmp(target, s_own_id) != 0) {
        ESP_LOGD(TAG, "[TRAJ] for %s (not us, skip)", target);
        return;
    }

    ESP_LOGI(TAG, "[TRAJ] target=%s  vel=(%.3f, %.3f) yaw=%.1f",
             target, vx, vy, yaw);

    vel_solver_set_trajectory_cmd(vx, vy, yaw);
}

/* ========================================================================
 * 初始化
 * ======================================================================*/

/**
 * @brief 初始化协议解析器。
 *
 * 必须在首次调用 proto_parse_message() 之前调用一次。
 * 会清空所有内部状态，并以"全连通、对角线为0"初始化默认拓扑矩阵
 * （即拓扑未到来之前，默认接受所有其他车辆的广播）。
 *
 * @param own_car_id 本车 ID 字符串，如 "CAR1"。
 */
void proto_parser_init(const char *own_car_id)
{
    strncpy(s_own_id, own_car_id, PROTO_CAR_ID_LEN - 1);
    s_own_id[PROTO_CAR_ID_LEN - 1] = '\0';
    strip_crlf(s_own_id);

    memset(&s_topology,      0, sizeof(s_topology));
    memset(&s_move_cmd,      0, sizeof(s_move_cmd));
    memset(&s_formation_cmd, 0, sizeof(s_formation_cmd));
    memset(s_cars,           0, sizeof(s_cars));

    /* 默认拓扑：全连通，自身不发给自身（对角线为 0） */
    for (int i = 0; i < PROTO_MAX_CARS; i++)
        for (int j = 0; j < PROTO_MAX_CARS; j++)
            s_topology.matrix[i][j] = (i == j) ? 0 : 1;

    ESP_LOGI(TAG, "init OK, own_id=%s", s_own_id);
}

/* ========================================================================
 * 拓扑 Accessor
 * ======================================================================*/

/**
 * @brief 返回当前拓扑配置的只读指针。
 *
 * 指针在下次 proto_parse_message() 调用前一直有效。
 * 外部仅读取，不得修改。
 */
const proto_topology_t *proto_get_topology(void)
{
    return &s_topology;
}

/**
 * @brief 判断是否应接受来自指定发送方的数据。
 *
 * 实现与 STM32 端 Should_Process_Car_Info() 相同的过滤规则：
 *   - 拓扑未使能 → 始终接受
 *   - 拓扑已使能 → matrix[sender_index][own_index] == 1 才接受
 *
 * 若 sender 或 own 的 ID 无法映射到有效索引（未知车辆），则默认接受。
 *
 * @param sender_car_id 发送方 ID，如 "CAR2"
 * @return true 接受，false 过滤丢弃
 */
bool proto_should_accept_from(const char *sender_car_id)
{
    if (!s_topology.enabled) return true;

    int si = proto_car_id_to_index(sender_car_id);
    int oi = proto_car_id_to_index(s_own_id);
    if (si < 0 || oi < 0) return true;   /* 未知车辆 → 默认接受 */

    return s_topology.matrix[si][oi] == 1;
}

/* ========================================================================
 * 移动指令 Accessor
 * ======================================================================*/

/**
 * @brief 消费最新的移动指令（一次性读取）。
 *
 * 调用后内部 valid 标志清除，同一条指令不会被二次消费。
 * 仅存储发给本车（s_own_id）的指令，其他车的移动指令会被丢弃。
 *
 * @param[out] out 指令数据填充目标
 * @return true 有新指令；false 无新指令
 */
bool proto_consume_move_cmd(proto_cmd_move_t *out)
{
    if (!s_move_cmd.valid) return false;
    *out = s_move_cmd;
    s_move_cmd.valid = false;
    return true;
}

/* ========================================================================
 * 编队指令 Accessor
 * ======================================================================*/

/**
 * @brief 消费最新的编队指令（一次性读取）。
 *
 * 调用后内部 valid 标志清除，同一条指令不会被二次消费。
 *
 * @param[out] out 指令数据填充目标
 * @return true 有新指令；false 无新指令
 */
bool proto_consume_formation_cmd(proto_cmd_formation_t *out)
{
    if (!s_formation_cmd.valid) return false;
    *out = s_formation_cmd;
    s_formation_cmd.valid = false;
    return true;
}


/* ========================================================================
 * 远端车辆状态 Accessor
 * ======================================================================*/

/**
 * @brief 查询指定车辆的最新状态。
 *
 * @param car_id 车辆 ID，如 "CAR2"
 * @return 指向内部状态结构体的只读指针；若未找到则返回 NULL。
 *         指针在下次 proto_parse_message() 前有效。
 */
const proto_car_state_t *proto_get_car_state(const char *car_id)
{
    for (int i = 0; i < PROTO_MAX_CARS; i++) {
        if (s_cars[i].valid && strcmp(s_cars[i].car_id, car_id) == 0)
            return &s_cars[i];
    }
    return NULL;
}

/**
 * @brief 获取全部已记录的远端车辆状态数组。
 *
 * 数组长度固定为 PROTO_MAX_CARS，未使用的槽位 valid==false。
 *
 * @param[out] count 有效条目数量（可传 NULL 忽略）
 * @return 指向内部数组首元素的只读指针
 */
const proto_car_state_t *proto_get_all_car_states(int *count)
{
    int n = 0;
    for (int i = 0; i < PROTO_MAX_CARS; i++)
        if (s_cars[i].valid) n++;
    if (count) *count = n;
    return s_cars;
}

/**
 * @brief 清除超时未更新的远端车辆条目。
 *
 * 建议在心跳任务中每秒调用一次，防止离线车辆的旧状态长期残留。
 *
 * @param timeout_ms 超时阈值（毫秒），典型值 3000
 */
void proto_cleanup_stale_cars(uint32_t timeout_ms)
{
    uint32_t now = now_ms();
    for (int i = 0; i < PROTO_MAX_CARS; i++) {
        if (s_cars[i].valid && (now - s_cars[i].last_update_ms) > timeout_ms) {
            ESP_LOGI(TAG, "remove stale car: %s", s_cars[i].car_id);
            memset(&s_cars[i], 0, sizeof(s_cars[i]));
        }
    }
}

/* ========================================================================
 * 内部辅助：更新远端车辆状态
 * ======================================================================*/

/**
 * @brief 将一辆远端车辆的状态写入 s_cars[] 表。
 *
 * 先检查拓扑过滤，若该发送方不在本车的接收列表中则丢弃。
 * 若表中已有该车的条目则原地更新，否则写入第一个空槽。
 * 若表已满则丢弃并打印警告。
 *
 * @param car_id 发送方 ID，如 "CAR2"
 * @param x,y    位置（米）
 * @param yaw    航向角（度）
 * @param vx,vy,vz 速度分量（米/秒 或 度/秒）
 */
static void update_car_state(const char *car_id,
                              float x, float y, float yaw,
                              float vx, float vy, float vz)
{
    /* 拓扑过滤：不在接收列表内则丢弃 */
    if (!proto_should_accept_from(car_id)) {
        ESP_LOGD(TAG, "topology blocked: %s", car_id);
        return;
    }

    /* 优先查找已有条目 */
    int slot = -1;
    for (int i = 0; i < PROTO_MAX_CARS; i++) {
        if (s_cars[i].valid && strcmp(s_cars[i].car_id, car_id) == 0) {
            slot = i;
            break;
        }
    }
    /* 未找到则分配空槽 */
    if (slot < 0) {
        for (int i = 0; i < PROTO_MAX_CARS; i++) {
            if (!s_cars[i].valid) { slot = i; break; }
        }
    }
    if (slot < 0) {
        ESP_LOGW(TAG, "car table full, drop %s", car_id);
        return;
    }

    strncpy(s_cars[slot].car_id, car_id, PROTO_CAR_ID_LEN - 1);
    s_cars[slot].x              = x;
    s_cars[slot].y              = y;
    s_cars[slot].yaw            = yaw;
    s_cars[slot].vx             = vx;
    s_cars[slot].vy             = vy;
    s_cars[slot].vz             = vz;
    s_cars[slot].last_update_ms = now_ms();
    s_cars[slot].valid          = true;
}

/* ========================================================================
 * 子解析器
 * ======================================================================*/

/* -----------------------------------------------------------------------
 * 消息类型 1 & 2：拓扑控制
 * 格式 1：[T,E,0|1]        拓扑过滤开关
 * 格式 2：[T,M,v00,...,v33] 4×4 拓扑矩阵（行主序，16 个 0/1 值）
 * ---------------------------------------------------------------------- */
static void parse_topology(const char *data)
{
    /* ---- 格式 1：[T,E,N] 使能/禁用 ---- */
    if (strncmp(data, "[T,E,", 5) == 0) {
        int en = atoi(data + 5);
        s_topology.enabled = (en != 0);

        ESP_LOGI(TAG, "[TOPO_ENABLE] enable=%d (%s)",
                 en, s_topology.enabled ? "ON" : "OFF");

        /* ================================================================
         * TODO: 拓扑使能变更处理点
         * 在此处通知 STM32 拓扑过滤已开启/关闭，例如：
         *   uart_bridge_send_topo_enable(s_topology.enabled);
         * ================================================================ */

        return;
    }

    /* ---- 格式 2：[T,M,...] 矩阵 ---- */
    if (strncmp(data, "[T,M,", 5) == 0) {
        int v[PROTO_MAX_CARS * PROTO_MAX_CARS];
        int n = sscanf(data,
            "[T,M,%d,%d,%d,%d,"
                  "%d,%d,%d,%d,"
                  "%d,%d,%d,%d,"
                  "%d,%d,%d,%d",
            &v[0],&v[1],&v[2],&v[3],
            &v[4],&v[5],&v[6],&v[7],
            &v[8],&v[9],&v[10],&v[11],
            &v[12],&v[13],&v[14],&v[15]);

        if (n == 16) {
            for (int i = 0; i < PROTO_MAX_CARS; i++)
                for (int j = 0; j < PROTO_MAX_CARS; j++)
                    s_topology.matrix[i][j] = (uint8_t)v[i * PROTO_MAX_CARS + j];
            s_topology.enabled = true;

            /* 打印完整矩阵，方便现场调试 */
            ESP_LOGI(TAG, "[TOPO_MATRIX] updated (auto-enabled):");
            for (int i = 0; i < PROTO_MAX_CARS; i++) {
                ESP_LOGI(TAG, "  row%d: %d %d %d %d", i,
                         s_topology.matrix[i][0], s_topology.matrix[i][1],
                         s_topology.matrix[i][2], s_topology.matrix[i][3]);
            }

            /* ============================================================
             * TODO: 拓扑矩阵更新处理点
             * 在此处将新矩阵下发给 STM32，例如：
             *   uart_bridge_send_topo_matrix(&s_topology);
             * ============================================================ */

        } else {
            ESP_LOGW(TAG, "[TOPO_MATRIX] parse failed (got %d/16 fields)", n);
        }
        return;
    }

    if (data[0] == '[' && data[1] == 'T' && data[2] == ',') {
        parse_trajectory_cmd(data);
        return;
    }

    ESP_LOGW(TAG, "[TOPO] unknown sub-cmd: %s", data);
}

/* -----------------------------------------------------------------------
 * 消息类型 3：点对点移动指令
 * 格式：[C,CARn,x,y,yaw]
 *   CARn - 目标车辆 ID（只处理发给本车的）
 *   x,y  - 目标位置（米）
 *   yaw  - 目标航向角（度）
 * ---------------------------------------------------------------------- */
static void parse_move_cmd(const char *data)
{
    if (recon_filter_controls()) {
        ESP_LOGW(TAG, "[MOVE] ignored during recon: %s", data);
        return;
    }

    char  target[PROTO_CAR_ID_LEN];
    float x, y, yaw;

    int n = sscanf(data, "[C,%9[^,],%f,%f,%f]", target, &x, &y, &yaw);
    if (n != 4) {
        ESP_LOGW(TAG, "[MOVE] parse failed: %s", data);
        return;
    }

    /* 仅保存发给本车的指令，其他车的丢弃 */
    if (strcmp(target, s_own_id) != 0) {
        ESP_LOGD(TAG, "[MOVE] for %s (not us, skip)", target);
        return;
    }

    strncpy(s_move_cmd.target_car, target, PROTO_CAR_ID_LEN - 1);
    s_move_cmd.x     = x;
    s_move_cmd.y     = y;
    s_move_cmd.yaw   = yaw;
    s_move_cmd.valid = true;

    ESP_LOGI(TAG, "[MOVE] target=%s  pos=(%.3f, %.3f)  yaw=%.1f deg",
             target, x, y, yaw);

    /* 解算模块：同步位置目标 */
    vel_solver_set_position_target(x, y, yaw);

    /* ====================================================================
     * TODO: 移动指令转发处理点
     * 在此处将移动指令通过 UART 下发给 STM32，例如：
     *   uart_bridge_send_move(x, y, yaw);
     * ==================================================================== */
}

/* -----------------------------------------------------------------------
 * 消息类型 4-8：编队控制指令
 * 子类型：
 *   [F,T]                        停止编队
 *   [F,S,CARn,type_str]          启动编队（指定 leader 和队形类型）
 *   [F,L,CARn]                   本车成为 Leader
 *   [F,F,CARn,off_x,off_y,oyaw]  本车成为 Follower，指定偏移量
 *   [F,U,CARn,off_x,off_y,oyaw]  更新跟随偏移量
 * ---------------------------------------------------------------------- */
static void parse_formation_cmd(const char *data)
{
    if (recon_filter_controls()) {
        ESP_LOGW(TAG, "[FORMATION] ignored during recon: %s", data);
        return;
    }

    if (strlen(data) < 4) return;
    char sub = data[3]; /* '[F,' 后的第一个字符 */

    proto_cmd_formation_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    switch (sub) {

        /* ---- 类型 4：停止编队 ---- */
        case 'T':
            cmd.type  = FORMATION_CMD_STOP;
            cmd.valid = true;
            ESP_LOGI(TAG, "[FORMATION] STOP");

            /* 解算模块：停止控制 */
            vel_solver_stop();

            /* ============================================================
             * TODO: 编队停止处理点
             * 在此处通知 STM32 停止编队控制，例如：
             *   uart_bridge_send_formation_stop();
             * ============================================================ */
            break;

        /* ---- 类型 5：启动编队 ---- */
        case 'S': {
            char leader[PROTO_CAR_ID_LEN], ftype[20];
            if (sscanf(data, "[F,S,%9[^,],%19[^]]", leader, ftype) == 2) {
                cmd.type = FORMATION_CMD_START;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.valid = true;
                ESP_LOGI(TAG, "[FORMATION] START  leader=%s  type=%s", leader, ftype);

                /* ========================================================
                 * TODO: 编队启动处理点
                 * 在此处通知 STM32 启动编队，例如：
                 *   uart_bridge_send_formation_start(leader, ftype);
                 * ======================================================== */
            }
            break;
        }

        /* ---- 类型 6：设置为 Leader ---- */
        case 'L': {
            char leader[PROTO_CAR_ID_LEN];
            if (sscanf(data, "[F,L,%9[^]]", leader) == 1) {
                cmd.type = FORMATION_CMD_SET_LEADER;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.valid = true;
                ESP_LOGI(TAG, "[FORMATION] SET_LEADER  car=%s", leader);

                /* 解算模块：进入 Leader 模式 */
                vel_solver_set_leader_mode();

                /* ========================================================
                 * TODO: 设置 Leader 处理点
                 * 在此处通知 STM32 本车担任 Leader，例如：
                 *   uart_bridge_send_set_leader();
                 * ======================================================== */
            }
            break;
        }

        /* ---- 类型 7：设置为 Follower ---- */
        case 'F': {
            char  leader[PROTO_CAR_ID_LEN];
            float ox, oy, oyaw;
            if (sscanf(data, "[F,F,%9[^,],%f,%f,%f", leader, &ox, &oy, &oyaw) == 4) {
                cmd.type       = FORMATION_CMD_SET_FOLLOWER;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.offset_x   = ox;
                cmd.offset_y   = oy;
                cmd.offset_yaw = oyaw;
                cmd.valid      = true;
                ESP_LOGI(TAG, "[FORMATION] SET_FOLLOWER  leader=%s  offset=(%.2f, %.2f, %.1f deg)",
                         leader, ox, oy, oyaw);

                /* 解算模块：进入 Follower 模式 */
                vel_solver_set_follower_mode(leader, ox, oy, oyaw);

                /* ========================================================
                 * TODO: 设置 Follower 处理点
                 * 在此处将 Follower 配置下发给 STM32，例如：
                 *   uart_bridge_send_set_follower(leader, ox, oy, oyaw);
                 * ======================================================== */
            }
            break;
        }

        /* ---- 类型 8：更新跟随偏移量 ---- */
        case 'U': {
            char  leader[PROTO_CAR_ID_LEN];
            float ox, oy, oyaw;
            if (sscanf(data, "[F,U,%9[^,],%f,%f,%f", leader, &ox, &oy, &oyaw) == 4) {
                cmd.type       = FORMATION_CMD_UPDATE_OFFSET;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.offset_x   = ox;
                cmd.offset_y   = oy;
                cmd.offset_yaw = oyaw;
                cmd.valid      = true;
                ESP_LOGI(TAG, "[FORMATION] UPDATE_OFFSET  leader=%s  offset=(%.2f, %.2f, %.1f deg)",
                         leader, ox, oy, oyaw);

                /* 解算模块：更新偏移 */
                vel_solver_update_follower_offset(ox, oy, oyaw);

                /* ========================================================
                 * TODO: 偏移量更新处理点
                 * 在此处将新偏移量下发给 STM32，例如：
                 *   uart_bridge_send_update_offset(leader, ox, oy, oyaw);
                 * ======================================================== */
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "[FORMATION] unknown sub-cmd: %c", sub);
            return;
    }

    if (cmd.valid) s_formation_cmd = cmd;
}

/* -----------------------------------------------------------------------
 * 消息类型 9：广播全场车辆状态
 * 格式：[N C1 x y yaw vx vy vz C2 x y yaw vx vy vz ...]
 *   N        - 本帧包含的车辆数量
 *   C1/C2... - 短 ID（"C1"→"CAR1"，"C2"→"CAR2"，以此类推）
 *   x,y      - 位置（米）
 *   yaw      - 航向角（度）
 *   vx,vy,vz - 速度分量
 * ---------------------------------------------------------------------- */
static void parse_broadcast_cars(const char *data)
{
    if (data[0] != '[') return;
    const char *p = data + 1;

    /* 解析车辆总数 */
    int car_count = 0;
    if (sscanf(p, "%d", &car_count) != 1) return;
    if (car_count <= 0 || car_count > PROTO_MAX_CARS) return;

    ESP_LOGD(TAG, "[BROADCAST] %d car(s) in frame", car_count);

    /* 跳过数量字段，指向第一个短 ID */
    p = strchr(p, ' ');
    if (!p) return;
    p++;

    for (int i = 0; i < car_count; i++) {
        char  short_id[4] = {0};
        float x, y, yaw, vx, vy, vz;

        int n = sscanf(p, "%3s %f %f %f %f %f %f",
                       short_id, &x, &y, &yaw, &vx, &vy, &vz);
        if (n != 7) break;

        /* 短 ID "C1" → "CAR1" */
        char car_id[PROTO_CAR_ID_LEN];
        snprintf(car_id, sizeof(car_id), "CAR%c", short_id[1]);

        /* 跳过自身数据（本车状态无需存储） */
        if (strcmp(car_id, s_own_id) == 0) {
            ESP_LOGD(TAG, "[BROADCAST] skip own (%s)", car_id);
        } else {
            update_car_state(car_id, x, y, yaw, vx, vy, vz);
            ESP_LOGI(TAG, "[BROADCAST] %s  pos=(%.2f,%.2f)  yaw=%.1f  vel=(%.2f,%.2f,%.2f)",
                     car_id, x, y, yaw, vx, vy, vz);

            /* 解算模块：通知车辆状态更新 */
            vel_solver_notify_car_updated(car_id);

            /* ============================================================
             * TODO: 广播车辆状态处理点
             * 在此处将其他车辆的状态转发给 STM32（用于编队算法），例如：
             *   uart_bridge_send_car_state(car_id, x, y, yaw, vx, vy, vz);
             * ============================================================ */
        }

        /* 指针前移 7 个 token（短 ID + 6 个浮点数） */
        for (int j = 0; j < 7; j++) {
            p = strchr(p, ' ');
            if (!p) break;
            p++;
        }
        if (!p || *p == '\0' || *p == ']') break;
    }
}

/* -----------------------------------------------------------------------
 * 消息类型 10：旧版拓扑矩阵（兼容格式）
 * 格式：TOPOLOGY:r0c0,r0c1,r0c2,r0c3;r1c0,...
 *   行与行之间用 ';' 分隔，同行内各元素用 ',' 分隔
 * ---------------------------------------------------------------------- */
static void parse_legacy_topology_matrix(const char *data)
{
    /* data 指向 "TOPOLOGY:" 后的第一个字符 */
    char buf[128];
    strncpy(buf, data, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int    row     = 0;
    char  *row_ptr = buf;

    while (row_ptr && *row_ptr && row < PROTO_MAX_CARS) {
        char *row_end = strchr(row_ptr, ';');
        if (row_end) *row_end = '\0';

        int   col     = 0;
        char *col_ptr = row_ptr;
        while (col_ptr && *col_ptr && col < PROTO_MAX_CARS) {
            while (*col_ptr == ' ' || *col_ptr == ',') col_ptr++;
            if (*col_ptr == '\0') break;
            s_topology.matrix[row][col] = (uint8_t)atoi(col_ptr);
            col++;
            col_ptr = strchr(col_ptr, ',');
            if (col_ptr) col_ptr++;
        }
        row++;
        row_ptr = row_end ? row_end + 1 : NULL;
    }
    s_topology.enabled = true;

    ESP_LOGI(TAG, "[TOPO_LEGACY] matrix updated (auto-enabled):");
    for (int i = 0; i < PROTO_MAX_CARS; i++) {
        ESP_LOGI(TAG, "  row%d: %d %d %d %d", i,
                 s_topology.matrix[i][0], s_topology.matrix[i][1],
                 s_topology.matrix[i][2], s_topology.matrix[i][3]);
    }

    /* ====================================================================
     * TODO: 旧版拓扑矩阵处理点
     * 在此处将更新后的矩阵下发给 STM32，例如：
     *   uart_bridge_send_topo_matrix(&s_topology);
     * ==================================================================== */
}

/* -----------------------------------------------------------------------
 * 消息类型 11：旧版拓扑使能切换（兼容格式）
 * 格式：TOPOLOGY_TOGGLE:True  或  TOPOLOGY_TOGGLE:False
 * ---------------------------------------------------------------------- */
static void parse_legacy_topology_toggle(const char *data)
{
    /* data 指向 "TOPOLOGY_TOGGLE:" 后的第一个字符 */
    bool en = (strncasecmp(data, "true", 4) == 0);
    s_topology.enabled = en;

    ESP_LOGI(TAG, "[TOPO_TOGGLE] -> %s", en ? "enabled" : "disabled");

    /* ====================================================================
     * TODO: 旧版拓扑切换处理点
     * 在此处通知 STM32 拓扑过滤已开启/关闭，例如：
     *   uart_bridge_send_topo_enable(en);
     * ==================================================================== */
}

/* -----------------------------------------------------------------------
 * 消息类型 12：手动遥控指令（Manual Control）
 * 格式：[M,CARn,vx,vy,vz]
 *   CARn - 目标车辆 ID（只处理发给本车的）
 *   vx   - 线速度 X（m/s，车头朝前为正）
 *   vy   - 线速度 Y（m/s，车头朝左为正）
 *   vz   - 角速度 Z（归一化 [-1,1]，逆时针为正）
 * ---------------------------------------------------------------------- */
static void parse_manual_cmd(const char *data)
{
    if (recon_filter_controls()) {
        ESP_LOGW(TAG, "[MANUAL] ignored during recon: %s", data);
        return;
    }

    char  target[PROTO_CAR_ID_LEN];
    float vx, vy, vz;

    int n = sscanf(data, "[M,%9[^,],%f,%f,%f]", target, &vx, &vy, &vz);
    if (n != 4) {
        ESP_LOGW(TAG, "[MANUAL] parse failed: %s", data);
        return;
    }

    /* 仅处理发给本车的指令 */
    if (strcmp(target, s_own_id) != 0) {
        ESP_LOGD(TAG, "[MANUAL] for %s (not us, skip)", target);
        return;
    }

    ESP_LOGI(TAG, "[MANUAL] target=%s  vel=(%.3f, %.3f, %.3f)", target, vx, vy, vz);

    /* 解算模块：直接透传手动速度 */
    vel_solver_set_manual_speed(vx, vy, vz);
}


/* ========================================================================
 * 主解析入口
 * ========================================================================
 * 根据消息头字符选择子解析器。
 * 括号格式 [X,...] 按第二个字符分发：
 *   T → 拓扑控制     C → 移动指令     F → 编队指令     M → 手动遥控
 *   0-9 → 广播车辆状态（数字表示车辆数）
 * 非括号格式按字符串前缀匹配旧版消息。
 * ======================================================================*/

void proto_parse_message(const char *data, size_t len)
{
    if (!data || len == 0) return;

    /* ---- 括号格式 [X,...] ---- */
    if (data[0] == '[' && len >= 4) {
        char type = data[1];
        switch (type) {
            case 'T': parse_topology(data);      return;
            case 'C': parse_move_cmd(data);      return;
            case 'F': parse_formation_cmd(data); return;
            case 'M': parse_manual_cmd(data);    return;
            default:  break;
        }
        /* 数字开头 → 广播车辆状态 */
        if (type >= '0' && type <= '9') {
            parse_broadcast_cars(data);
            return;
        }
    }

    /* ---- 旧版字符串前缀消息 ---- */
    if (strncmp(data, "TOPOLOGY_TOGGLE:", 16) == 0) {
        parse_legacy_topology_toggle(data + 16);
        return;
    }
    if (strncmp(data, "TOPOLOGY:", 9) == 0) {
        parse_legacy_topology_matrix(data + 9);
        return;
    }

    /* 未能识别的消息，仅在 DEBUG 级别打印（避免刷屏） */
    ESP_LOGD(TAG, "unhandled msg: %.*s", (int)(len > 40 ? 40 : len), data);
}
