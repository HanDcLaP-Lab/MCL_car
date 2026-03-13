#include "imu_car_rc.h"
IMU_Car_Data_t imu_car_data = {0}; 

void IMU_Car_RC_Init(void){
    while(1)
    {
         if(imu660rc_init(IMU660RC_QUARTERNION_480HZ))                          // 设置 imu660rc 以120HZ的速度产生中断触发信号
        {
           printf("\r\n imu660rc init error.");                                 // imu660rc 初始化失败
        }
        else
        {
           break;
        }
    }
}

void IMU_Car_RC_Update_Loop(void){
   imu_data.roll = imu660rc_roll;
   imu_data.pitch = imu660rc_pitch;
   imu_data.yaw = imu660rc_yaw;
   imu_data.is_calibrated = 1;
}