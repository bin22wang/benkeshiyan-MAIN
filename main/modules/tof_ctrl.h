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

// Initialize dual VL53L0X sensors (I2C + XSHUT + calibration-ready).
esp_err_t tof_ctrl_init(void);

// Read calibrated distance in mm (applies linear calibration).
esp_err_t tof_ctrl_read(tof_dual_reading_t *out);

// Read raw distance in mm (no calibration).
esp_err_t tof_ctrl_read_raw(tof_dual_reading_t *out);

#ifdef __cplusplus
}
#endif
