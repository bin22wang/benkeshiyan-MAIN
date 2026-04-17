#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t magic;
    float left_scale;
    float left_bias;
    float right_scale;
    float right_bias;
} calib_params_t;

esp_err_t calib_store_init(void);

bool calib_store_load(calib_params_t *out);

esp_err_t calib_store_save(const calib_params_t *params);

#ifdef __cplusplus
}
#endif
