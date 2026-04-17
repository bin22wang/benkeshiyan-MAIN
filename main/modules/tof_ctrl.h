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

// Get the latest cached reading and age in milliseconds (if available).
// Returns ESP_OK only if a valid reading has been recorded.
esp_err_t tof_ctrl_get_latest(tof_dual_reading_t *out, uint32_t *age_ms);

// Update calibration parameters (scale/bias) for both sensors.
void tof_ctrl_set_calibration(float left_scale, float left_bias,
                              float right_scale, float right_bias);

// Read current calibration parameters.
void tof_ctrl_get_calibration(float *left_scale, float *left_bias,
                              float *right_scale, float *right_bias);

#ifdef __cplusplus
}
#endif
