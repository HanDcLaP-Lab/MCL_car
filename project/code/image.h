#ifndef _IMAGE_H
#define _IMAGE_H

#include "zf_common_headfile.h"

// UART接收的数据数组
// 0: Car X, 1: Car Y, 2: Target X, 3: Target Y
// 4: Roll, 5: Pitch, 6: Yaw, 7: Height
extern float uart_data[8];

void Image_Init(void);
// 解算目标相对于小车的距离和方位角
void Image_Solve(float car_yaw, float *dist, float *angle);

#endif