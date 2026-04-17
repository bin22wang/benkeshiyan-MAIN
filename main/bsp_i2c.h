#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t bsp_i2c_master_init(void);
esp_err_t bsp_i2c_aux_init(void);

#ifdef __cplusplus
}
#endif
