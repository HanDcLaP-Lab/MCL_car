#ifndef _CAR_IMAGE_H
#define _CAR_IMAGE_H

#include "zf_common_headfile.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 无人机下传 + 车端计算综合状态结构体
// uart_data[] 接收后通过 car_stat_update() 一次性写入，其他模块仅读取 car.*
typedef struct {
    // 无人机下传原始值 (无人机坐标系, cm, deg)
    float car_x;            // [0] 小车地面X坐标
    float car_y;            // [1] 小车地面Y坐标
    float target_x;         // [2] 目标地面X坐标 (raw)
    float target_y;         // [3] 目标地面Y坐标 (raw)
    float drone_yaw;        // [4] 无人机偏航角
    float locked_state;     // [5] 锁定状态 (0=全丢/1=仅小车/2=仅信标/3=都有)
    float car_en;           // [6] 急停使能标志 (0=急停, 1=正常)

    // 车端滤波值
    float target_x_f;       // 低通滤波后的目标X
    float target_y_f;       // 低通滤波后的目标Y
    float car_target_dist;  // 车-目标距离 (cm)
} car_stat_t;

extern car_stat_t car;

void Image_Init(void);
// 从 uart_data[] 填充 car 结构体并执行低通滤波和距离计算
void car_stat_update(void);
// 解算目标相对于小车的距离和方位角 (使用 car.target_x_f/y_f)
void Image_Solve(float car_yaw, float *dist, float *angle);

#endif