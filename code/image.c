#include "image.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
}

// ==========================================
// 5. 计算最终夹角
// ==========================================
static double calculateFinalAngle(double dx, double dy, double yaw_cam, double yaw_car) {
    // 1. 在无人机局部坐标系下，目标相对于小车的夹角
    double angle_in_drone_frame = atan2(dx, dy) * 180.0 / M_PI;
    
    // 2. 加入绝对偏航角修正
    double final_angle = angle_in_drone_frame + (yaw_cam - yaw_car);
    
    // 3. 归一化到 [0, 360) 区间
    while (final_angle < 0) final_angle += 360.0;
    while (final_angle >= 360.0) final_angle -= 360.0;
    
    return final_angle;
}

// ==========================================
// 6. 核心解算接口
// ==========================================
void Image_Solve(float car_yaw, float *dist, float *angle) {
    // 直接获取无人机解算好的物理坐标 (单位: 米)
    double x_car = (double)uart_data[0];
    double y_car = (double)uart_data[1];
    double x_target = (double)uart_data[2];
    double y_target = (double)uart_data[3];
    double yaw_drone = (double)uart_data[4];
    double yaw_car = (double)car_yaw;

    double dx = x_target - x_car;
    double dy = y_target - y_car;
    
    *dist = (float)sqrt(dx * dx + dy * dy);
    *angle = (float)calculateFinalAngle(dx, dy, yaw_drone, yaw_car);
}