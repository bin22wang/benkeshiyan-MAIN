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

typedef struct {
    float fx;
    float fy;
    float cx;
    float cy;
    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
} camera_intrinsics_t;

void aruco_pose_init(void);
bool aruco_pose_get_latest(aruco_pose_t *out);
const camera_intrinsics_t *aruco_pose_get_intrinsics(void);

#ifdef __cplusplus
}
#endif
