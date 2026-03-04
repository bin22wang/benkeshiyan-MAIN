/**
 * @file proto_parser.c
 * @brief Protocol parser implementation
 *
 * Parses all incoming UDP message formats and stores results in
 * internal state variables accessible via the accessor API.
 */

#include "proto_parser.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "proto";

/* ========== Internal State ========== */

static char s_own_id[PROTO_CAR_ID_LEN] = "CAR1";

static proto_topology_t     s_topology;
static proto_cmd_move_t     s_move_cmd;
static proto_cmd_formation_t s_formation_cmd;
static proto_car_state_t    s_cars[PROTO_MAX_CARS];

/* ========== Utility ========== */

int proto_car_id_to_index(const char *car_id)
{
    if (strcmp(car_id, "CAR1") == 0) return 0;
    if (strcmp(car_id, "CAR2") == 0) return 1;
    if (strcmp(car_id, "CAR3") == 0) return 2;
    if (strcmp(car_id, "CAR4") == 0) return 3;
    return -1;
}

/* Strip trailing \r \n and whitespace in-place */
static void strip_crlf(char *s)
{
    int len = (int)strlen(s);
    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' '))
        s[--len] = '\0';
}

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* ========== Initialisation ========== */

void proto_parser_init(const char *own_car_id)
{
    strncpy(s_own_id, own_car_id, PROTO_CAR_ID_LEN - 1);
    s_own_id[PROTO_CAR_ID_LEN - 1] = '\0';
    strip_crlf(s_own_id);

    memset(&s_topology,     0, sizeof(s_topology));
    memset(&s_move_cmd,     0, sizeof(s_move_cmd));
    memset(&s_formation_cmd,0, sizeof(s_formation_cmd));
    memset(s_cars,          0, sizeof(s_cars));

    /* Default topology: full-mesh, diagonal = 0 */
    for (int i = 0; i < PROTO_MAX_CARS; i++)
        for (int j = 0; j < PROTO_MAX_CARS; j++)
            s_topology.matrix[i][j] = (i == j) ? 0 : 1;

    ESP_LOGI(TAG, "init, own_id=%s", s_own_id);
}

/* ========== Topology Accessors ========== */

const proto_topology_t *proto_get_topology(void)
{
    return &s_topology;
}

bool proto_should_accept_from(const char *sender_car_id)
{
    if (!s_topology.enabled) return true;

    int si = proto_car_id_to_index(sender_car_id);
    int oi = proto_car_id_to_index(s_own_id);
    if (si < 0 || oi < 0) return true;   /* unknown car ¡ú accept */

    return s_topology.matrix[si][oi] == 1;
}

/* ========== Move Command ========== */

bool proto_consume_move_cmd(proto_cmd_move_t *out)
{
    if (!s_move_cmd.valid) return false;
    *out = s_move_cmd;
    s_move_cmd.valid = false;
    return true;
}

/* ========== Formation Command ========== */

bool proto_consume_formation_cmd(proto_cmd_formation_t *out)
{
    if (!s_formation_cmd.valid) return false;
    *out = s_formation_cmd;
    s_formation_cmd.valid = false;
    return true;
}

/* ========== Remote Car State ========== */

const proto_car_state_t *proto_get_car_state(const char *car_id)
{
    for (int i = 0; i < PROTO_MAX_CARS; i++) {
        if (s_cars[i].valid && strcmp(s_cars[i].car_id, car_id) == 0)
            return &s_cars[i];
    }
    return NULL;
}

const proto_car_state_t *proto_get_all_car_states(int *count)
{
    int n = 0;
    for (int i = 0; i < PROTO_MAX_CARS; i++)
        if (s_cars[i].valid) n++;
    if (count) *count = n;
    return s_cars;
}

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

/* ========== Internal helpers ========== */

/* Update (or insert) a remote car's state */
static void update_car_state(const char *car_id,
                              float x, float y, float yaw,
                              float vx, float vy, float vz)
{
    /* topology filter */
    if (!proto_should_accept_from(car_id)) {
        ESP_LOGD(TAG, "topology blocked: %s", car_id);
        return;
    }

    /* find existing slot */
    int slot = -1;
    for (int i = 0; i < PROTO_MAX_CARS; i++) {
        if (s_cars[i].valid && strcmp(s_cars[i].car_id, car_id) == 0) {
            slot = i;
            break;
        }
    }
    /* find empty slot */
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

/* ========== Sub-parsers ========== */

/**
 * [T,E,0|1]
 * [T,M,v00,v01,v02,v03,v10,...,v33]
 */
static void parse_topology(const char *data)
{
    if (strncmp(data, "[T,E,", 5) == 0) {
        int en = atoi(data + 5);
        s_topology.enabled = (en != 0);
        ESP_LOGI(TAG, "topology %s", s_topology.enabled ? "enabled" : "disabled");
        return;
    }

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
            ESP_LOGI(TAG, "topology matrix updated (auto-enabled)");
        } else {
            ESP_LOGW(TAG, "topology matrix parse failed (got %d/16)", n);
        }
        return;
    }

    ESP_LOGW(TAG, "unknown topology cmd: %s", data);
}

/**
 * [C,CARn,x,y,yaw]
 */
static void parse_move_cmd(const char *data)
{
    char target[PROTO_CAR_ID_LEN];
    float x, y, yaw;

    int n = sscanf(data, "[C,%9[^,],%f,%f,%f]", target, &x, &y, &yaw);
    if (n != 4) {
        ESP_LOGW(TAG, "move cmd parse failed: %s", data);
        return;
    }

    /* only store if addressed to this car */
    if (strcmp(target, s_own_id) != 0) {
        ESP_LOGD(TAG, "move cmd for %s (not us), skip", target);
        return;
    }

    strncpy(s_move_cmd.target_car, target, PROTO_CAR_ID_LEN - 1);
    s_move_cmd.x     = x;
    s_move_cmd.y     = y;
    s_move_cmd.yaw   = yaw;
    s_move_cmd.valid = true;
    ESP_LOGI(TAG, "move cmd -> (%.3f, %.3f, %.1f deg)", x, y, yaw);
}

/**
 * [F,T]
 * [F,S,CARn,type]
 * [F,L,CARn]
 * [F,F,CARn,off_x,off_y,off_yaw]
 * [F,U,CARn,off_x,off_y,off_yaw]
 */
static void parse_formation_cmd(const char *data)
{
    if (strlen(data) < 4) return;
    char sub = data[3]; /* char after "[F," */

    proto_cmd_formation_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    switch (sub) {
        case 'T': /* stop */
            cmd.type  = FORMATION_CMD_STOP;
            cmd.valid = true;
            ESP_LOGI(TAG, "formation STOP");
            break;

        case 'S': { /* [F,S,CARn,type_str] */
            char leader[PROTO_CAR_ID_LEN], ftype[20];
            if (sscanf(data, "[F,S,%9[^,],%19[^]]", leader, ftype) == 2) {
                cmd.type = FORMATION_CMD_START;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.valid = true;
                ESP_LOGI(TAG, "formation START, leader=%s type=%s", leader, ftype);
            }
            break;
        }

        case 'L': { /* [F,L,CARn] */
            char leader[PROTO_CAR_ID_LEN];
            if (sscanf(data, "[F,L,%9[^]]", leader) == 1) {
                cmd.type = FORMATION_CMD_SET_LEADER;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.valid = true;
                ESP_LOGI(TAG, "formation SET_LEADER: %s", leader);
            }
            break;
        }

        case 'F': { /* [F,F,CARn,ox,oy,oyaw] */
            char leader[PROTO_CAR_ID_LEN];
            float ox, oy, oyaw;
            if (sscanf(data, "[F,F,%9[^,],%f,%f,%f", leader, &ox, &oy, &oyaw) == 4) {
                cmd.type       = FORMATION_CMD_SET_FOLLOWER;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.offset_x   = ox;
                cmd.offset_y   = oy;
                cmd.offset_yaw = oyaw;
                cmd.valid      = true;
                ESP_LOGI(TAG, "formation FOLLOWER of %s, offset=(%.2f,%.2f,%.1f)",
                         leader, ox, oy, oyaw);
            }
            break;
        }

        case 'U': { /* [F,U,CARn,ox,oy,oyaw] */
            char leader[PROTO_CAR_ID_LEN];
            float ox, oy, oyaw;
            if (sscanf(data, "[F,U,%9[^,],%f,%f,%f", leader, &ox, &oy, &oyaw) == 4) {
                cmd.type       = FORMATION_CMD_UPDATE_OFFSET;
                strncpy(cmd.leader_id, leader, PROTO_CAR_ID_LEN - 1);
                cmd.offset_x   = ox;
                cmd.offset_y   = oy;
                cmd.offset_yaw = oyaw;
                cmd.valid      = true;
                ESP_LOGI(TAG, "formation UPDATE_OFFSET for %s -> (%.2f,%.2f,%.1f)",
                         leader, ox, oy, oyaw);
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "unknown formation sub-cmd: %c", sub);
            return;
    }

    if (cmd.valid) s_formation_cmd = cmd;
}

/**
 * Compact broadcast: [N C1 x y yaw vx vy vz C2 ...]
 * N = number of cars following
 * Short ID "C1" maps to "CAR1", "C2"->"CAR2", etc.
 */
static void parse_broadcast_cars(const char *data)
{
    if (data[0] != '[') return;
    const char *p = data + 1;

    int car_count = 0;
    if (sscanf(p, "%d", &car_count) != 1) return;
    if (car_count <= 0 || car_count > PROTO_MAX_CARS) return;

    /* advance past the count number */
    p = strchr(p, ' ');
    if (!p) return;
    p++;

    for (int i = 0; i < car_count; i++) {
        char short_id[4] = {0};
        float x, y, yaw, vx, vy, vz;

        int n = sscanf(p, "%3s %f %f %f %f %f %f",
                       short_id, &x, &y, &yaw, &vx, &vy, &vz);
        if (n != 7) break;

        /* "C1" -> "CAR1" */
        char car_id[PROTO_CAR_ID_LEN];
        snprintf(car_id, sizeof(car_id), "CAR%c", short_id[1]);

        /* skip own data */
        if (strcmp(car_id, s_own_id) != 0) {
            update_car_state(car_id, x, y, yaw, vx, vy, vz);
        }

        /* advance pointer past 7 tokens */
        for (int j = 0; j < 7; j++) {
            p = strchr(p, ' ');
            if (!p) break;
            p++;
        }
        if (!p || *p == '\0' || *p == ']') break;
    }
}

/**
 * Legacy: TOPOLOGY:r0c0,r0c1,...,r0c3;r1c0,...
 */
static void parse_legacy_topology_matrix(const char *data)
{
    /* data points just past "TOPOLOGY:" */
    char buf[128];
    strncpy(buf, data, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int row = 0;
    char *row_ptr = buf;

    while (row_ptr && *row_ptr && row < PROTO_MAX_CARS) {
        char *row_end = strchr(row_ptr, ';');
        if (row_end) *row_end = '\0';

        int col = 0;
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
    ESP_LOGI(TAG, "legacy topology matrix updated");
}

/**
 * Legacy: TOPOLOGY_TOGGLE:True|False
 */
static void parse_legacy_topology_toggle(const char *data)
{
    /* data points just past "TOPOLOGY_TOGGLE:" */
    bool en = (strncasecmp(data, "true", 4) == 0);
    s_topology.enabled = en;
    ESP_LOGI(TAG, "topology toggle -> %s", en ? "enabled" : "disabled");
}

/* ========== Main Entry ========== */

void proto_parse_message(const char *data, size_t len)
{
    if (!data || len == 0) return;

    /* --- Bracket-format messages [X,...] --- */
    if (data[0] == '[' && len >= 4) {
        char type = data[1];
        switch (type) {
            case 'T': parse_topology(data);      return;
            case 'C': parse_move_cmd(data);      return;
            case 'F': parse_formation_cmd(data); return;
            default: break;
        }
        /* Numeric after '[' ¡ú broadcast car states */
        if (type >= '0' && type <= '9') {
            parse_broadcast_cars(data);
            return;
        }
    }

    /* --- Legacy string-prefix messages --- */
    if (strncmp(data, "TOPOLOGY_TOGGLE:", 16) == 0) {
        parse_legacy_topology_toggle(data + 16);
        return;
    }
    if (strncmp(data, "TOPOLOGY:", 9) == 0) {
        parse_legacy_topology_matrix(data + 9);
        return;
    }

    ESP_LOGD(TAG, "unhandled msg: %.*s", (int)(len > 40 ? 40 : len), data);
}
