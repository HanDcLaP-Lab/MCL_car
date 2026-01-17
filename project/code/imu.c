#include "imu.h"

// ================= 全局变量定义 =================
car_angle = 0;
float gyro_measureVal;
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
     gyro_measureVal = imu660ra_gyro_transition(imu660ra_gyro_z) * IMU_DT;
     car_angle += Kalman_Update(&K_w , gyro_measureVal);
}

void IMU_Init(){
    while(1)///定时器0初始化
    {
        if(imu660ra_init())
        {
           printf("\r\n imu660ra init error.");                                 // imu660ra 初始化失败
        }
        else
        {
           break;
        }
        //gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
    Kalman_Init(&K_w , 0.01f,0.01,0);
}