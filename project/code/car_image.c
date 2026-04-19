#include "car_image.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
}
void Image_Solve(float car_yaw, float *dist, float *angle) {
    extern float uart_data[8];
    // 直接获取无人机解算好的物理坐标 (单位: cm)
    double x_car = (double)uart_data[0];
    double y_car = (double)uart_data[1];
    double x_target = (double)uart_data[2];
    double y_target = (double)uart_data[3];
    double yaw_drone = (double)uart_data[4];

    // 1. 计算无人机坐标系下的相对矢量 (X:前, Y:右)
    double dx = x_target - x_car;
    double dy = y_target - y_car;
    
    // 2. 计算距离并转换为 cm (适配新的PID参数)
    *dist = (float)sqrt(dx * dx + dy * dy);

    // 3. 坐标系旋转: 无人机坐标系 -> 小车坐标系
    // 下传数据X前Y右，小车X前Y右，Yaw均为顺时针正
    // 旋转角 delta = Car_Yaw - Drone_Yaw
    double delta_rad = ((double)car_yaw - yaw_drone) * (M_PI / 180.0);

    // 旋转矩阵 (顺时针旋转坐标系/逆时针旋转向量)
    // dx_car = dx * cos(delta) + dy * sin(delta)
    // dy_car = -dx * sin(delta) + dy * cos(delta)
    double dx_car = dx * cos(delta_rad) + dy * sin(delta_rad);
    double dy_car = -dx * sin(delta_rad) + dy * cos(delta_rad);

    // 4. 计算角度 (0度为车头, 90度为车右, 符合 atan2(y, x) 定义)
    *angle = (float)(atan2(dy_car, dx_car) * 180.0 / M_PI);
}