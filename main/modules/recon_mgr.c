#include "recon_mgr.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "udp_comm.h"

#define RECON_CAR_ID_LEN 10
#define RECON_ORDER_LEN  4

static const char *TAG = "recon_mgr";

static recon_state_t s_state = RECON_STATE_IDLE;
static char s_own_id[RECON_CAR_ID_LEN] = "CAR1";
static char s_order[RECON_ORDER_LEN][RECON_CAR_ID_LEN] = {{0}};
static recon_pos_t s_last_pos = RECON_POS_UNKNOWN;

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

static void set_state(recon_state_t next)
{
    if (s_state != next) {
        ESP_LOGI(TAG, "state %s -> %s", recon_mgr_state_str(s_state), recon_mgr_state_str(next));
        s_state = next;
    }
}

static void handle_prep(void)
{
    set_state(RECON_STATE_PREPARING);
    ESP_LOGI(TAG, "PREP received: placeholder action complete");

    char msg[32];
    snprintf(msg, sizeof(msg), "[R,PREP_OK,%s]", s_own_id);
    recon_send(msg);
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

    if (strcmp(target, s_own_id) != 0) {
        ESP_LOGD(TAG, "STEP for %s (not us)", target);
        return;
    }

    recon_pos_t pos = parse_pos(pos_text);
    s_last_pos = pos;

    if (is_separating) {
        set_state(RECON_STATE_SEPARATING);
        ESP_LOGI(TAG, "SEP_STEP for %s pos=%s: placeholder action complete", target, recon_mgr_pos_str(pos));

        char msg[48];
        snprintf(msg, sizeof(msg), "[R,SEP_OK,%s,POS=%s]", s_own_id, recon_mgr_pos_str(pos));
        recon_send(msg);
    } else {
        set_state(RECON_STATE_ASSEMBLING);
        ESP_LOGI(TAG, "STEP for %s pos=%s: placeholder action complete", target, recon_mgr_pos_str(pos));

        char msg[48];
        snprintf(msg, sizeof(msg), "[R,STEP_OK,%s,POS=%s]", s_own_id, recon_mgr_pos_str(pos));
        recon_send(msg);
    }
}

static void handle_done(void)
{
    set_state(RECON_STATE_ASSEMBLED);
}

static void handle_sep_begin(void)
{
    set_state(RECON_STATE_SEPARATING);
}

static void handle_sep_done(void)
{
    set_state(RECON_STATE_IDLE);
}

static void handle_abort(void)
{
    ESP_LOGW(TAG, "ABORT received: stop action, reset to IDLE");
    set_state(RECON_STATE_IDLE);
}

void recon_mgr_init(const char *own_car_id)
{
    if (own_car_id && own_car_id[0] != '\0') {
        strncpy(s_own_id, own_car_id, RECON_CAR_ID_LEN - 1);
        s_own_id[RECON_CAR_ID_LEN - 1] = '\0';
    }
    set_state(RECON_STATE_IDLE);
    memset(s_order, 0, sizeof(s_order));
    s_last_pos = RECON_POS_UNKNOWN;
    ESP_LOGI(TAG, "init OK, own_id=%s", s_own_id);
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
