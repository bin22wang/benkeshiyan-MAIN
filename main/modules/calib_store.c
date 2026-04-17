#include "calib_store.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "at24c02.h"

#define CALIB_MAGIC 0x43414C31u
#define CALIB_EEPROM_ADDR 0x00

static const char *TAG = "calib_store";

esp_err_t calib_store_init(void)
{
    return at24c02_init();
}

bool calib_store_load(calib_params_t *out)
{
    if (!out) {
        return false;
    }

    calib_params_t temp = {0};
    for (int attempt = 0; attempt < 2; attempt++) {
        memset(&temp, 0, sizeof(temp));
        at24c02_read(CALIB_EEPROM_ADDR, (uint8_t *)&temp, sizeof(temp));
        if (temp.magic == CALIB_MAGIC) {
            *out = temp;
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "calib magic mismatch: 0x%08X", (unsigned)temp.magic);
    return false;
}

esp_err_t calib_store_save(const calib_params_t *params)
{
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }

    calib_params_t temp = *params;
    temp.magic = CALIB_MAGIC;
    for (int attempt = 0; attempt < 2; attempt++) {
        at24c02_write(CALIB_EEPROM_ADDR, (uint8_t *)&temp, sizeof(temp));
        vTaskDelay(pdMS_TO_TICKS(20));

        calib_params_t verify = {0};
        at24c02_read(CALIB_EEPROM_ADDR, (uint8_t *)&verify, sizeof(verify));
        if (verify.magic == CALIB_MAGIC) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGW(TAG, "calib save verify failed");
    return ESP_FAIL;
}
