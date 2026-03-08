#ifndef _CAR_IMAGE_H
#define _CAR_IMAGE_H

#include "zf_common_headfile.h"

// UART接收的数据数组
// 0: Car X, 1: Car Y (无人机坐标系下的地面坐标)
// 2: Target X, 3: Target Y (无人机坐标系下的地面坐标)
// 4: Drone Yaw (无人机偏航角)


void Image_Init(void);
// 解算目标相对于小车的距离和方位角
void Image_Solve(float car_yaw, float *dist, float *angle);

#endif