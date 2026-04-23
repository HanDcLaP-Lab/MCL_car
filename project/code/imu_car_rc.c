#include "imu_car_rc.h"
IMU_Car_RC_Data_t imu_car_rc_data = {0}; 

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
    // =========================================================
    // 1. 坐标系轴向映射 (保持你修改好的正常逻辑)
    // =========================================================
    imu_car_rc_data.roll = -imu660rc_pitch;
   
    if(imu660rc_roll > 90.0) imu_car_rc_data.pitch = imu660rc_roll - 180.0f;
    if(imu660rc_roll < -90.0) imu_car_rc_data.pitch = imu660rc_roll + 180.0f;

    // =========================================================
    // 2. Yaw 基础角度处理 (-180 到 180 范围)
    // =========================================================
    float new_yaw = imu660rc_yaw; 
    
    // 官方驱动解算出的 imu660rc_yaw 范围通常是 0~360，将其转为 -180~180
    if (new_yaw > 180.0f) {
        new_yaw -= 360.0f;
    }

    // =========================================================
    // 3. 多圈角度累计 (yaw_total) 核心逻辑
    // =========================================================
    if (imu_car_rc_data.is_calibrated != 0) {
        // 计算本周期与上个周期的偏航角差值
        float yaw_diff = new_yaw - imu_car_rc_data.yaw;
        
        // 【关键】处理 -180 和 180 交界处的跳变
        // 比如从 179 跳变到 -179 时，差值为 -358，需要加 360 修正为真实的 2度 增量
        if (yaw_diff < -180.0f) {
            yaw_diff += 360.0f;
        } else if (yaw_diff > 180.0f) {
            yaw_diff -= 360.0f;
        }
        
        // 持续累加到总角度
        imu_car_rc_data.yaw_total += yaw_diff;
    } else {
        // 第一次上电循环，给 yaw_total 赋初始值，防止启动时跳变
        imu_car_rc_data.yaw_total = new_yaw; 
    }

    // 更新当前帧的 yaw 供下一次差值计算使用
    imu_car_rc_data.yaw = new_yaw;

    // =========================================================
    // 4. 获取 Yaw Rate (角速度) - 供给 PID 的 D 项
    // =========================================================
    imu_car_rc_data.yaw_rate = imu660rc_gyro_transition(imu660rc_gyro_z);

    // 标记初始化已完成
    imu_car_rc_data.is_calibrated = 1;
}