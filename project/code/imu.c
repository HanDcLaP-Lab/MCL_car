#include "imu.h"

// ================= 全局变量定义 =================
car_angle = 0;
// ================= 内部辅助函数 =================
// 快速平方根倒数
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


void IMU_Update_Loop(void){
    car_angle += imu660ra_gyro_transition(imu660ra_gyro_z) * IMU_DT;
}