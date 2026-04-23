#include "recon_mgr.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "udp_comm.h"
#include "vel_solver.h"
#include "proto_parser.h"

#define RECON_CAR_ID_LEN 10
#define RECON_ORDER_LEN  4

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    uint32_t task_interval_ms;
    float prep_spacing_m;
    float prep_pos_tol_m;
    float prep_yaw_tol_deg;
    float tof_deadband_mm;
    float tof_align_limit_mm;
    float tof_vy_k;
    float tof_vy_max;
    float tof_vx_max;
    float tof_vx_min;
    float tof_yaw_k;
    float tof_vz_max;
    float tof_dock_min_mm;
    float tof_dock_max_mm;
    uint32_t sep_wait_ms;
    uint32_t sep_post_back_ms;
    float sep_success_mm;
    float sep_back_vx;
    float sep_yaw_k;
    float sep_vz_max;
} recon_cfg_t;

static const recon_cfg_t k_recon_cfg = {
    .task_interval_ms = 50,
    .prep_spacing_m = 0.50f,
    .prep_pos_tol_m = 0.07f,
    .prep_yaw_tol_deg = 2.0f,
    .tof_deadband_mm = 5.0f,
    .tof_align_limit_mm = 30.0f,
    .tof_vy_k = 0.0025f,
    .tof_vy_max = 0.12f,
    .tof_vx_max = 0.06f,
    .tof_vx_min = 0.02f,
    .tof_yaw_k = 0.02f,
    .tof_vz_max = 0.30f,
    .tof_dock_min_mm = 20.0f,
    .tof_dock_max_mm = 30.0f,
    .sep_wait_ms = 3000,
    .sep_post_back_ms = 3500,
    .sep_success_mm = 40.0f,
    .sep_back_vx = 0.05f,
    .sep_yaw_k = 0.02f,
    .sep_vz_max = 0.30f,
};

static const char *TAG = "recon_mgr";

static bool is_front_of_target(const char *target_id);
static void recon_update_magnet(bool enable);
static int find_order_index(const char *car_id);

static recon_state_t s_state = RECON_STATE_IDLE;
static char s_own_id[RECON_CAR_ID_LEN] = "CAR1";
static char s_order[RECON_ORDER_LEN][RECON_CAR_ID_LEN] = {{0}};
static recon_pos_t s_last_pos = RECON_POS_UNKNOWN;
static bool s_order_valid = false;
static bool s_prep_reported = false;
static bool s_step_active = false;
static char s_front_id[RECON_CAR_ID_LEN] = {0};
static char s_sep_front_id[RECON_CAR_ID_LEN] = {0};
static TaskHandle_t s_task = NULL;
static uint32_t s_last_prep_set_ms = 0;

typedef enum {
    SEP_ROLE_NONE = 0,
    SEP_ROLE_FRONT,
    SEP_ROLE_REAR,
} sep_role_t;

typedef enum {
    SEP_PHASE_IDLE = 0,
    SEP_PHASE_WAIT,
    SEP_PHASE_BACK,
    SEP_PHASE_POST,
} sep_phase_t;

static sep_role_t s_sep_role = SEP_ROLE_NONE;
static sep_phase_t s_sep_phase = SEP_PHASE_IDLE;
static uint32_t s_sep_start_ms = 0;
static uint32_t s_sep_phase_ms = 0;
static bool s_sep_reported = false;
static bool s_sep_magnet_on = false;

static bool recon_send(const char *msg)
{
    if (!msg) return false;
    size_t len = strlen(msg);
    if (len == 0) return false;
    bool ok = udp_comm_send(msg, len);
    if (!ok) {
        ESP_LOGW(TAG, "send failed: %s", msg);
    }
    return ok;
}

const char *recon_mgr_state_str(recon_state_t state)
{
    switch (state) {
        case RECON_STATE_IDLE: return "idle";
        case RECON_STATE_PREPARING: return "preparing";
        case RECON_STATE_ASSEMBLING: return "assembling";
        case RECON_STATE_ASSEMBLED: return "assembled";
        case RECON_STATE_SEPARATING: return "separating";
        case RECON_STATE_ERROR: return "error";
        default: return "unknown";
    }
}

const char *recon_mgr_pos_str(recon_pos_t pos)
{
    switch (pos) {
        case RECON_POS_HEAD: return "HEAD";
        case RECON_POS_MID2: return "MID2";
        case RECON_POS_MID3: return "MID3";
        case RECON_POS_TAIL: return "TAIL";
        default: return "UNKNOWN";
    }
}

static recon_pos_t parse_pos(const char *pos)
{
    if (!pos) return RECON_POS_UNKNOWN;
    if (strcmp(pos, "HEAD") == 0) return RECON_POS_HEAD;
    if (strcmp(pos, "MID2") == 0) return RECON_POS_MID2;
    if (strcmp(pos, "MID3") == 0) return RECON_POS_MID3;
    if (strcmp(pos, "TAIL") == 0) return RECON_POS_TAIL;
    return RECON_POS_UNKNOWN;
}

static float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float wrap_deg(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static void set_state(recon_state_t next)
{
    if (s_state != next) {
        ESP_LOGI(TAG, "state %s -> %s", recon_mgr_state_str(s_state), recon_mgr_state_str(next));
        s_state = next;
    }
    if (next == RECON_STATE_PREPARING) {
        s_prep_reported = false;
    }
    if (next != RECON_STATE_ASSEMBLING) {
        s_step_active = false;
    }
    if (next != RECON_STATE_SEPARATING) {
        s_sep_role = SEP_ROLE_NONE;
        s_sep_phase = SEP_PHASE_IDLE;
        s_sep_reported = false;
        s_sep_start_ms = 0;
        s_sep_phase_ms = 0;
        s_sep_front_id[0] = '\0';
        if (s_sep_magnet_on) {
            s_sep_magnet_on = false;
            ESP_LOGI(TAG, "magnet off");
        }
    }
}

static void handle_prep(void)
{
    set_state(RECON_STATE_PREPARING);
    s_last_prep_set_ms = 0;
    ESP_LOGI(TAG, "PREP received: start alignment with UWB");
}

static void handle_order(const char *data)
{
    char c1[RECON_CAR_ID_LEN] = {0};
    char c2[RECON_CAR_ID_LEN] = {0};
    char c3[RECON_CAR_ID_LEN] = {0};
    char c4[RECON_CAR_ID_LEN] = {0};

    int n = sscanf(data, "[R,ORDER,%9[^,],%9[^,],%9[^,],%9[^]]]", c1, c2, c3, c4);
    if (n != 4) {
        ESP_LOGW(TAG, "ORDER parse failed: %s", data);
        return;
    }

    strncpy(s_order[0], c1, RECON_CAR_ID_LEN - 1);
    strncpy(s_order[1], c2, RECON_CAR_ID_LEN - 1);
    strncpy(s_order[2], c3, RECON_CAR_ID_LEN - 1);
    strncpy(s_order[3], c4, RECON_CAR_ID_LEN - 1);
    s_order_valid = true;

    ESP_LOGI(TAG, "ORDER set: %s %s %s %s", s_order[0], s_order[1], s_order[2], s_order[3]);
}

static void handle_step(const char *data, bool is_separating)
{
    char target[RECON_CAR_ID_LEN] = {0};
    char pos_text[16] = {0};

    int n = sscanf(data, "[R,%*[^,],%9[^,],POS=%15[^]]]", target, pos_text);
    if (n != 2) {
        ESP_LOGW(TAG, "STEP parse failed: %s", data);
        return;
    }

    if (is_separating) {
        bool is_front = is_front_of_target(target);
        if (is_front) {
            recon_update_magnet(true);
        } else if (s_sep_magnet_on) {
            recon_update_magnet(false);
        }
    }

    if (strcmp(target, s_own_id) != 0) {
        if (is_separating) {
            ESP_LOGD(TAG, "SEP_STEP for %s (not us)", target);
        } else {
            ESP_LOGD(TAG, "STEP for %s (not us)", target);
        }
        return;
    }

    recon_pos_t pos = parse_pos(pos_text);
    s_last_pos = pos;

    if (is_separating) {
        set_state(RECON_STATE_SEPARATING);
        s_sep_role = SEP_ROLE_REAR;
        s_sep_phase = SEP_PHASE_WAIT;
        s_sep_start_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        s_sep_phase_ms = s_sep_start_ms;
        s_sep_reported = false;
        s_sep_front_id[0] = '\0';

        if (s_order_valid) {
            int idx = find_order_index(s_own_id);
            if (idx > 0) {
                strncpy(s_sep_front_id, s_order[idx - 1], RECON_CAR_ID_LEN - 1);
                s_sep_front_id[RECON_CAR_ID_LEN - 1] = '\0';
            }
        }

        ESP_LOGI(TAG, "SEP_STEP for %s pos=%s: wait %ums then back, front=%s",
                 target,
                 recon_mgr_pos_str(pos),
                 (unsigned int)k_recon_cfg.sep_wait_ms,
                 s_sep_front_id[0] ? s_sep_front_id : "UNKNOWN");
    } else {
        set_state(RECON_STATE_ASSEMBLING);
        s_step_active = true;
        s_front_id[0] = '\0';

        if (pos == RECON_POS_HEAD) {
            ESP_LOGI(TAG, "STEP for %s pos=%s: head no docking, OK", target, recon_mgr_pos_str(pos));
            char msg[48];
            snprintf(msg, sizeof(msg), "[R,STEP_OK,%s,POS=%s]", s_own_id, recon_mgr_pos_str(pos));
            recon_send(msg);
            set_state(RECON_STATE_ASSEMBLED);
            s_step_active = false;
            return;
        }

        if (s_order_valid) {
            for (int i = 0; i < RECON_ORDER_LEN; i++) {
                if (strcmp(s_order[i], s_own_id) == 0) {
                    if (i > 0) {
                        strncpy(s_front_id, s_order[i - 1], RECON_CAR_ID_LEN - 1);
                        s_front_id[RECON_CAR_ID_LEN - 1] = '\0';
                    }
                    break;
                }
            }
        }

        ESP_LOGI(TAG, "STEP for %s pos=%s: start ToF docking, front=%s",
                 target, recon_mgr_pos_str(pos), s_front_id[0] ? s_front_id : "UNKNOWN");
    }
}

static void handle_done(void)
{
    set_state(RECON_STATE_ASSEMBLED);
    vel_solver_stop();
}

static void handle_sep_begin(void)
{
    set_state(RECON_STATE_SEPARATING);
}

static void handle_sep_done(void)
{
    set_state(RECON_STATE_IDLE);
    vel_solver_stop();
}

static void handle_abort(void)
{
    ESP_LOGW(TAG, "ABORT received: stop action, reset to IDLE");
    set_state(RECON_STATE_IDLE);
    vel_solver_stop();
}

static int find_order_index(const char *car_id)
{
    if (!s_order_valid || !car_id) return -1;
    for (int i = 0; i < RECON_ORDER_LEN; i++) {
        if (strcmp(s_order[i], car_id) == 0) return i;
    }
    return -1;
}

static bool is_front_of_target(const char *target_id)
{
    if (!s_order_valid || !target_id) return false;
    int target_idx = find_order_index(target_id);
    if (target_idx <= 0) return false;
    return strcmp(s_order[target_idx - 1], s_own_id) == 0;
}

static void recon_handle_preparing(void)
{
    if (!s_order_valid) {
        return;
    }

    int idx = find_order_index(s_own_id);
    if (idx < 0) return;

    const vel_own_state_t *own = vel_solver_get_own_state();
    if (!own->valid) return;

    if (idx == 0) {
        vel_solver_stop();
        if (!s_prep_reported) {
            char msg[32];
            snprintf(msg, sizeof(msg), "[R,PREP_OK,%s]", s_own_id);
            recon_send(msg);
            s_prep_reported = true;
            ESP_LOGI(TAG, "PREP OK (head)");
        }
        return;
    }

    const proto_car_state_t *head = proto_get_car_state(s_order[0]);
    if (!head || !head->valid) {
        return;
    }

    float head_yaw = head->yaw;
    float yaw_rad = head_yaw * (float)M_PI / 180.0f;
    float spacing = k_recon_cfg.prep_spacing_m * (float)idx;

    float target_x = head->x - cosf(yaw_rad) * spacing;
    float target_y = head->y - sinf(yaw_rad) * spacing;
    float target_yaw = head_yaw;

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - s_last_prep_set_ms >= 200) {
        vel_solver_set_position_target(target_x, target_y, target_yaw);
        s_last_prep_set_ms = now;
    }

    float dx = target_x - own->x;
    float dy = target_y - own->y;
    float dist = sqrtf(dx * dx + dy * dy);
    float yaw_err = wrap_deg(target_yaw - own->yaw);

    if (!s_prep_reported && dist <= k_recon_cfg.prep_pos_tol_m && fabsf(yaw_err) <= k_recon_cfg.prep_yaw_tol_deg) {
        char msg[32];
        snprintf(msg, sizeof(msg), "[R,PREP_OK,%s]", s_own_id);
        recon_send(msg);
        s_prep_reported = true;
        ESP_LOGI(TAG, "PREP OK (idx=%d dist=%.3f yaw_err=%.1f)", idx, dist, yaw_err);
    }
}

static void recon_update_magnet(bool enable)
{
    if (s_sep_magnet_on == enable) return;
    s_sep_magnet_on = enable;
    ESP_LOGI(TAG, "magnet %s", enable ? "on" : "off");
}

static void recon_handle_separating(void)
{
    if (s_sep_role == SEP_ROLE_FRONT) {
        recon_update_magnet(true);
        vel_solver_stop();
        return;
    }

    if (s_sep_role != SEP_ROLE_REAR) {
        return;
    }

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (s_sep_phase == SEP_PHASE_WAIT) {
        if (now - s_sep_start_ms >= k_recon_cfg.sep_wait_ms) {
            s_sep_phase = SEP_PHASE_BACK;
            s_sep_phase_ms = now;
        } else {
            vel_solver_stop();
            return;
        }
    }

    float vx = -k_recon_cfg.sep_back_vx;
    float vy = 0.0f;
    float vz = 0.0f;

    const vel_own_state_t *own = vel_solver_get_own_state();
    if (own->valid && s_sep_front_id[0] != '\0') {
        const proto_car_state_t *front = proto_get_car_state(s_sep_front_id);
        if (front && front->valid) {
            float yaw_err = wrap_deg(front->yaw - own->yaw);
            vz = clamp_float(yaw_err * k_recon_cfg.sep_yaw_k,
                             -k_recon_cfg.sep_vz_max,
                             k_recon_cfg.sep_vz_max);
        }
    }

    if (s_sep_phase == SEP_PHASE_BACK) {
        if (!s_sep_reported && (now - s_sep_phase_ms) >= k_recon_cfg.sep_post_back_ms) {
            s_sep_reported = true;
            s_sep_phase = SEP_PHASE_POST;
            s_sep_phase_ms = now;
            ESP_LOGW(TAG, "separation timed out without ToF, enter post phase");
        }
    }

    if (s_sep_phase == SEP_PHASE_POST) {
        if (now - s_sep_phase_ms >= k_recon_cfg.sep_post_back_ms) {
            vel_solver_stop();
            s_sep_phase = SEP_PHASE_IDLE;
            return;
        }
    }

    vel_solver_set_manual_speed(vx, vy, vz);
}

static void recon_handle_assembling(void)
{
    if (!s_step_active) return;

    if (s_last_pos == RECON_POS_HEAD) {
        return;
    }

    const vel_own_state_t *own = vel_solver_get_own_state();
    if (!own->valid) {
        return;
    }

    const proto_car_state_t *front = NULL;
    if (s_front_id[0] != '\0') {
        front = proto_get_car_state(s_front_id);
    }

    float target_yaw = own->yaw;
    if (front && front->valid) {
        target_yaw = front->yaw;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    float yaw_err = wrap_deg(target_yaw - own->yaw);
    vz = clamp_float(yaw_err * k_recon_cfg.tof_yaw_k,
                     -k_recon_cfg.tof_vz_max,
                     k_recon_cfg.tof_vz_max);

    vel_solver_set_manual_speed(vx, vy, vz);
}

static void recon_mgr_task(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        switch (s_state) {
            case RECON_STATE_PREPARING:
                recon_handle_preparing();
                break;
            case RECON_STATE_ASSEMBLING:
                recon_handle_assembling();
                break;
            case RECON_STATE_SEPARATING:
                recon_handle_separating();
                break;
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(k_recon_cfg.task_interval_ms));
    }
}

void recon_mgr_init(const char *own_car_id)
{
    if (own_car_id && own_car_id[0] != '\0') {
        strncpy(s_own_id, own_car_id, RECON_CAR_ID_LEN - 1);
        s_own_id[RECON_CAR_ID_LEN - 1] = '\0';
    }
    set_state(RECON_STATE_IDLE);
    memset(s_order, 0, sizeof(s_order));
    s_order_valid = false;
    s_prep_reported = false;
    s_step_active = false;
    s_front_id[0] = '\0';
    s_sep_front_id[0] = '\0';
    s_sep_role = SEP_ROLE_NONE;
    s_sep_phase = SEP_PHASE_IDLE;
    s_sep_start_ms = 0;
    s_sep_phase_ms = 0;
    s_sep_reported = false;
    s_sep_magnet_on = false;
    s_last_prep_set_ms = 0;
    s_last_pos = RECON_POS_UNKNOWN;
    ESP_LOGI(TAG, "init OK, own_id=%s", s_own_id);

    recon_mgr_start();
}

recon_state_t recon_mgr_get_state(void)
{
    return s_state;
}

void recon_mgr_handle_message(const char *data, size_t len)
{
    if (!data || len < 4) return;
    if (data[0] != '[' || data[1] != 'R' || data[2] != ',') return;

    if (strncmp(data, "[R,PREP]", 8) == 0) {
        handle_prep();
        return;
    }
    if (strncmp(data, "[R,ORDER,", 9) == 0) {
        handle_order(data);
        return;
    }
    if (strncmp(data, "[R,STEP,", 8) == 0) {
        handle_step(data, false);
        return;
    }
    if (strncmp(data, "[R,DONE]", 8) == 0) {
        handle_done();
        return;
    }
    if (strncmp(data, "[R,SEP_BEGIN]", 13) == 0) {
        handle_sep_begin();
        return;
    }
    if (strncmp(data, "[R,SEP_STEP,", 12) == 0) {
        handle_step(data, true);
        return;
    }
    if (strncmp(data, "[R,SEP_DONE]", 12) == 0) {
        handle_sep_done();
        return;
    }
    if (strncmp(data, "[R,ABORT]", 9) == 0) {
        handle_abort();
        return;
    }

    ESP_LOGW(TAG, "unknown R cmd: %.*s", (int)(len > 60 ? 60 : len), data);
}

void recon_mgr_start(void)
{
    if (s_task != NULL) return;
    xTaskCreate(recon_mgr_task, "recon_task", 4096, NULL, 4, &s_task);
}
