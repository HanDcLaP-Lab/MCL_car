#ifndef _CAR_IMAGE_H
#define _CAR_IMAGE_H

#include "zf_common_headfile.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// UART接收的数据数组 uart_data[8] — 完整映射见 car_board_comm.h
// 0: Car X, 1: Car Y (无人机坐标系下的地面坐标, cm)
// 2: Target X, 3: Target Y (无人机坐标系下的地面坐标, cm)
// 4: Drone Yaw (无人机偏航角, deg, 顺时针正)
// 5: locked_state (目标锁定状态: 0=全丢/1=仅小车/2=仅信标/3=都有)
// 6: car_en (急停使能标志, 0=停 1=行)
// 7: car_target_dist (车-信标地面距离, cm)


void Image_Init(void);
// 解算目标相对于小车的距离和方位角
void Image_Solve(float car_yaw, float *dist, float *angle);

#endif