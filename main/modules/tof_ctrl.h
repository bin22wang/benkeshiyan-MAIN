#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t left_mm;
    uint16_t right_mm;
    bool left_ok;
    bool right_ok;
} tof_dual_reading_t;

esp_err_t tof_ctrl_init(void);

esp_err_t tof_ctrl_read(tof_dual_reading_t *out);

#ifdef __cplusplus
}
#endif
