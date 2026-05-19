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
    RECON_STATE_READY,
    RECON_STATE_ASSEMBLING,
    RECON_STATE_GUIDING,
    RECON_STATE_SUCCESS,
} recon_state_t;

void recon_mgr_init(const char *own_car_id);

recon_state_t recon_mgr_get_state(void);

const char *recon_mgr_state_str(recon_state_t state);

void recon_mgr_handle_message(const char *data, size_t len);

// Optional: start background control task (auto-started in recon_mgr_init).
void recon_mgr_start(void);

#ifdef __cplusplus
}
#endif
