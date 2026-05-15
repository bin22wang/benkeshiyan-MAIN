#include "recon_mgr.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "proto_parser.h"
#include "udp_comm.h"
#include "vel_solver.h"

#define RECON_CAR_ID_LEN      10
#define RECON_ORDER_LEN       4
#define RECON_TASK_INTERVAL_MS 50
#define RECON_PREP_SPACING_M  0.70f
#define RECON_PREP_UPDATE_MS  200

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "recon_mgr";

static recon_state_t s_state = RECON_STATE_IDLE;
static char s_own_id[RECON_CAR_ID_LEN] = "CAR1";
static char s_order[RECON_ORDER_LEN][RECON_CAR_ID_LEN] = {{0}};
static int s_order_count = 0;
static bool s_order_valid = false;
static char s_front_id[RECON_CAR_ID_LEN] = {0};
static uint32_t s_last_prep_set_ms = 0;
static TaskHandle_t s_task = NULL;

static bool recon_send(const char *msg)
{
    if (!msg || msg[0] == '\0') return false;
    size_t len = strlen(msg);
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
        case RECON_STATE_GUIDING: return "guiding";
        case RECON_STATE_WAITING: return "waiting";
        case RECON_STATE_ERROR: return "error";
        default: return "unknown";
    }
}

static void set_state(recon_state_t next)
{
    if (s_state != next) {
        ESP_LOGI(TAG, "state %s -> %s", recon_mgr_state_str(s_state), recon_mgr_state_str(next));
        s_state = next;
    }
}

static void clear_order(void)
{
    memset(s_order, 0, sizeof(s_order));
    s_order_count = 0;
    s_order_valid = false;
    s_front_id[0] = '\0';
}

static int find_order_index(const char *car_id)
{
    if (!s_order_valid || !car_id) return -1;
    for (int i = 0; i < s_order_count; i++) {
        if (strcmp(s_order[i], car_id) == 0) return i;
    }
    return -1;
}

static void update_front_id(void)
{
    s_front_id[0] = '\0';
    int idx = find_order_index(s_own_id);
    if (idx > 0) {
        strncpy(s_front_id, s_order[idx - 1], RECON_CAR_ID_LEN - 1);
        s_front_id[RECON_CAR_ID_LEN - 1] = '\0';
    }
}

static void handle_prep(void)
{
    set_state(RECON_STATE_PREPARING);
    clear_order();
    s_last_prep_set_ms = 0;
    vel_solver_stop();

    char msg[32];
    snprintf(msg, sizeof(msg), "[R,PREP_OK,%s]", s_own_id);
    recon_send(msg);
    ESP_LOGI(TAG, "PREP received -> PREP_OK sent");
}

static void handle_order(const char *data)
{
    if (!data) return;

    const char *start = strstr(data, "[R,ORDER,");
    if (!start) return;
    start += strlen("[R,ORDER,");

    const char *end = strchr(start, ']');
    if (!end || end <= start) {
        ESP_LOGW(TAG, "ORDER parse failed: %s", data);
        return;
    }

    char buf[128];
    size_t copy_len = (size_t)(end - start);
    if (copy_len >= sizeof(buf)) copy_len = sizeof(buf) - 1;
    memcpy(buf, start, copy_len);
    buf[copy_len] = '\0';

    clear_order();

    char *saveptr = NULL;
    char *token = strtok_r(buf, ",", &saveptr);
    while (token && s_order_count < RECON_ORDER_LEN) {
        while (*token == ' ') token++;
        if (*token != '\0') {
            strncpy(s_order[s_order_count], token, RECON_CAR_ID_LEN - 1);
            s_order[s_order_count][RECON_CAR_ID_LEN - 1] = '\0';
            s_order_count++;
        }
        token = strtok_r(NULL, ",", &saveptr);
    }

    s_order_valid = (s_order_count > 0);
    update_front_id();

    ESP_LOGI(TAG, "ORDER set (%d entries)", s_order_count);
}

static void handle_step(const char *data)
{
    if (!data) return;
    char target[RECON_CAR_ID_LEN] = {0};

    int n = sscanf(data, "[R,STEP,%9[^,]", target);
    if (n != 1) {
        ESP_LOGW(TAG, "STEP parse failed: %s", data);
        return;
    }

    if (strcmp(target, s_own_id) != 0) {
        return;
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "[R,STEP_ACK,%s]", s_own_id);
    recon_send(msg);
    set_state(RECON_STATE_GUIDING);
    update_front_id();
    ESP_LOGI(TAG, "STEP for %s -> STEP_ACK", target);
}

static void handle_guide(const char *data)
{
    if (!data) return;
    char target[RECON_CAR_ID_LEN] = {0};
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;
    int done = 0;

    int n = sscanf(data, "[R,GUIDE,%9[^,],VX=%f,VY=%f,VZ=%f,DONE=%d]",
                   target, &vx, &vy, &vz, &done);
    if (n != 5) {
        ESP_LOGW(TAG, "GUIDE parse failed: %s", data);
        return;
    }

    if (strcmp(target, s_own_id) != 0) {
        return;
    }

    if (s_state != RECON_STATE_GUIDING && s_state != RECON_STATE_WAITING) {
        ESP_LOGW(TAG, "GUIDE ignored in state=%s", recon_mgr_state_str(s_state));
        return;
    }

    if (done == 0) {
        set_state(RECON_STATE_GUIDING);
        vel_solver_set_manual_speed(vx, vy, vz);
        return;
    }

    vel_solver_stop();

    char msg[32];
    snprintf(msg, sizeof(msg), "[R,STEP_OK,%s]", s_own_id);
    recon_send(msg);
    set_state(RECON_STATE_WAITING);
    ESP_LOGI(TAG, "GUIDE DONE -> STEP_OK");
}

static void handle_abort_done(const char *reason)
{
    ESP_LOGW(TAG, "%s received: stop and reset", reason);
    vel_solver_stop();
    clear_order();
    set_state(RECON_STATE_IDLE);
}

static void recon_handle_preparing(void)
{
    if (!s_order_valid) {
        vel_solver_stop();
        return;
    }

    int idx = find_order_index(s_own_id);
    if (idx < 0) {
        vel_solver_stop();
        return;
    }

    if (idx == 0) {
        vel_solver_stop();
        return;
    }

    const proto_car_state_t *front = proto_get_car_state(s_order[idx - 1]);
    if (!front || !front->valid) {
        vel_solver_stop();
        return;
    }

    float yaw_rad = front->yaw * (float)M_PI / 180.0f;
    float target_x = front->x - cosf(yaw_rad) * RECON_PREP_SPACING_M;
    float target_y = front->y - sinf(yaw_rad) * RECON_PREP_SPACING_M;
    float target_yaw = front->yaw;

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - s_last_prep_set_ms >= RECON_PREP_UPDATE_MS) {
        vel_solver_set_position_target(target_x, target_y, target_yaw);
        s_last_prep_set_ms = now;
    }
}

static void recon_mgr_task(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        if (s_state == RECON_STATE_PREPARING) {
            recon_handle_preparing();
        }
        vTaskDelay(pdMS_TO_TICKS(RECON_TASK_INTERVAL_MS));
    }
}

void recon_mgr_init(const char *own_car_id)
{
    if (own_car_id && own_car_id[0] != '\0') {
        strncpy(s_own_id, own_car_id, RECON_CAR_ID_LEN - 1);
        s_own_id[RECON_CAR_ID_LEN - 1] = '\0';
    }

    set_state(RECON_STATE_IDLE);
    clear_order();
    s_last_prep_set_ms = 0;
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
        handle_step(data);
        return;
    }
    if (strncmp(data, "[R,GUIDE,", 9) == 0) {
        handle_guide(data);
        return;
    }
    if (strncmp(data, "[R,ABORT]", 9) == 0) {
        handle_abort_done("ABORT");
        return;
    }
    if (strncmp(data, "[R,DONE]", 8) == 0) {
        handle_abort_done("DONE");
        return;
    }

    ESP_LOGW(TAG, "unknown R cmd: %.*s", (int)(len > 60 ? 60 : len), data);
}

void recon_mgr_start(void)
{
    if (s_task != NULL) return;
    xTaskCreate(recon_mgr_task, "recon_task", 4096, NULL, 4, &s_task);
}