#include "car_image.h"
#include <math.h>
#include <string.h>




void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
}
void Image_Solve(float car_yaw, float *dist, float *angle) {
    extern float uart_data[8];
    // 直接获取无人机解算好的物理坐标 (单位: cm)
    float x_car = uart_data[0];       // [0] 小车地面X坐标
    float y_car = uart_data[1];       // [1] 小车地面Y坐标
    float x_target = uart_data[2];    // [2] 目标地面X坐标
    float y_target = uart_data[3];    // [3] 目标地面Y坐标
    float yaw_drone = uart_data[4];   // [4] 无人机偏航角 (deg)

    // 1. 计算无人机坐标系下的相对矢量 (X:前, Y:右)
    float dx = x_target - x_car;
    float dy = y_target - y_car;
    
    // 2. 计算距离 (cm)
    *dist = sqrtf(dx * dx + dy * dy);

    // 3. 坐标系旋转: 无人机坐标系 -> 小车坐标系
    // 下传数据X前Y右，小车X前Y左，Yaw均为顺时针正
    // 旋转角 delta = Car_Yaw - Drone_Yaw
    float delta_rad = (car_yaw - yaw_drone) * ((float)(M_PI / 180.0));

    // 旋转矩阵 (顺时针旋转坐标系/逆时针旋转向量)
    // dx_car = dx * cos(delta) + dy * sin(delta)
    // dy_car = -dx * sin(delta) + dy * cos(delta)
    float dx_car = dx * cosf(delta_rad) + dy * sinf(delta_rad);
    float dy_car = -dx * sinf(delta_rad) + dy * cosf(delta_rad);

    dy_car = -dy_car; // Y轴取反适配小车坐标系

    // 4. 计算角度 (0度为车头, 90度为车右, 符合 atan2(y, x) 定义)
    *angle = atan2f(dy_car, dx_car) * (180.0f / (float)M_PI);
}