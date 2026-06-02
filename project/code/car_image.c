#include "car_image.h"
#include <math.h>
#include <string.h>


car_stat_t car = {0};

void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
    memset(&car, 0, sizeof(car));
}

void car_stat_update(void) {
    const float k = 0.8f;

    // 复制无人机下传原始值
    car.car_x        = uart_data[0];
    car.car_y        = uart_data[1];
    car.target_x     = uart_data[2];
    car.target_y     = uart_data[3];
    car.drone_yaw    = uart_data[4];
    car.locked_state = uart_data[5];
    car.car_en       = uart_data[6];

    // 低通滤波目标坐标 (不修改 car.target_x/y 原始值)
    car.target_x_f = car.target_x_f * (1.0f - k) + car.target_x * k;
    car.target_y_f = car.target_y_f * (1.0f - k) + car.target_y * k;

    // 车-目标距离
    float dx = car.car_x - car.target_x_f;
    float dy = car.car_y - car.target_y_f;
    car.car_target_dist = sqrtf(dx * dx + dy * dy);
}

void Image_Solve(float car_yaw, float *dist, float *angle) {
    // 使用滤波后的目标坐标
    float x_car = car.car_x;
    float y_car = car.car_y;
    float x_target = car.target_x_f;
    float y_target = car.target_y_f;
    float yaw_drone = car.drone_yaw;

    // 1. 计算无人机坐标系下的相对矢量 (X:前, Y:右)
    float dx = x_target - x_car;
    float dy = y_target - y_car;

    // 2. 计算距离 (cm)
    *dist = sqrtf(dx * dx + dy * dy);

    // 3. 坐标系旋转: 无人机坐标系 -> 小车坐标系
    float delta_rad = (car_yaw - yaw_drone) * ((float)(M_PI / 180.0));

    float dx_car = dx * cosf(delta_rad) + dy * sinf(delta_rad);
    float dy_car = -dx * sinf(delta_rad) + dy * cosf(delta_rad);

    dy_car = -dy_car;

    // 4. 计算角度
    *angle = atan2f(dy_car, dx_car) * (180.0f / (float)M_PI);
}
