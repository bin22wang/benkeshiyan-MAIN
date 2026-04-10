#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RECON_STATE_IDLE = 0,
    RECON_STATE_PREPARING,
    RECON_STATE_ASSEMBLING,
    RECON_STATE_ASSEMBLED,
    RECON_STATE_SEPARATING,
    RECON_STATE_ERROR,
} recon_state_t;

typedef enum {
    RECON_POS_UNKNOWN = 0,
    RECON_POS_HEAD,
    RECON_POS_MID2,
    RECON_POS_MID3,
    RECON_POS_TAIL,
} recon_pos_t;

void recon_mgr_init(const char *own_car_id);

recon_state_t recon_mgr_get_state(void);

const char *recon_mgr_state_str(recon_state_t state);
const char *recon_mgr_pos_str(recon_pos_t pos);

void recon_mgr_handle_message(const char *data, size_t len);

// Optional: start background control task (auto-started in recon_mgr_init).
void recon_mgr_start(void);

#ifdef __cplusplus
}
#endif
