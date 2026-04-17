#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t calib_mgr_init(void);
void calib_mgr_start(void);
void calib_mgr_set_active(bool active);
bool calib_mgr_is_active(void);

#ifdef __cplusplus
}
#endif
