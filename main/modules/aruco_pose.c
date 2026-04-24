#include "aruco_pose.h"

#include "disp_mgr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

static aruco_pose_t s_pose = {0};

static void aruco_pose_task(void *pvParameters)
{
    (void)pvParameters;

    float phase = 0.0f;

    while (1) {
        if (disp_mgr_get_page() == DISP_PAGE_CAMERA) {
            phase += 0.08f;
            if (phase > 6.283185f) {
                phase -= 6.283185f;
            }

            s_pose.x = 0.25f * cosf(phase);
            s_pose.y = 0.25f * sinf(phase);
            s_pose.yaw = 30.0f * sinf(phase * 0.7f);
            s_pose.conf = 0.85f;
            s_pose.valid = true;
        } else {
            s_pose.valid = false;
            s_pose.conf = 0.0f;
        }

        disp_mgr_update_aruco(s_pose.x, s_pose.y, s_pose.yaw, s_pose.conf, s_pose.valid);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void aruco_pose_init(void)
{
    memset(&s_pose, 0, sizeof(s_pose));
    xTaskCreate(aruco_pose_task, "aruco_pose_task", 3072, NULL, 3, NULL);
}

bool aruco_pose_get_latest(aruco_pose_t *out)
{
    if (!out) {
        return false;
    }

    *out = s_pose;
    return true;
}
