#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float yaw;
    float conf;
    bool valid;
} aruco_pose_t;

void aruco_pose_init(void);
bool aruco_pose_get_latest(aruco_pose_t *out);

#ifdef __cplusplus
}
#endif
