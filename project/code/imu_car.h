#ifndef _IMU_CAR_H
#define _IMU_CAR_H

#include "zf_common_headfile.h" // 保持你的底层头文件
#include <math.h>
#include <stdint.h>

// ================= 配置参数 =================
// 麦轮小车震动比无人机大，建议适当降低 KP (更信任陀螺仪)，提高 KI (消除长期漂移)
#define IMU_KP 0.5f          // 比例增益
#define IMU_KI 0.005f        // 积分增益
#define IMU_DT 0.001f        // 运行周期 1ms (1000Hz)
#define GRAVITY_MSS 9.789f   // 标准重力加速度
#define VALID_G_MIN 0.5f     // 陀螺仪死区 (小车静止时通常比稳，可设小)

// ================= 坐标系映射宏定义 =================
// 目标: 车体坐标系 (X前, Y右, Z下) - 符合右手定则
// 传感器安装: X向左, Y向前, Z向下

// 陀螺仪映射 (机体角速度)
#define IMU_MAP_GX(x, y, z)  (y)
#define IMU_MAP_GY(x, y, z)  (-x)
#define IMU_MAP_GZ(x, y, z)  (z)

// 加速度计映射 (映射到重力向量方向，即 -1 * 机体加速度)
#define IMU_MAP_AX(x, y, z)  (y)
#define IMU_MAP_AY(x, y, z)  (-x)
#define IMU_MAP_AZ(x, y, z)  (-z)

#ifndef PI
#define PI 3.1415926535f
#endif

// ================= 核心结构体定义 =================
typedef struct {
    // --- 姿态角 (单位: 度) ---
    float roll;
    float pitch;
    float yaw;          // 范围 -180 ~ +180

    // --- 扩展控制数据 ---
    float yaw_total;    // 范围 -inf ~ +inf (连续累计角度，用于多圈控制)
    float yaw_rate;     // Z轴角速度 (度/秒)，用于 PID D项，极重要！

    // --- 运动监测 (单位: m/s^2) ---
    // 已转换到世界坐标系 (去除重力)
    // 麦轮小车可用此数据检测撞击 (Impact Detection)
    float world_acc_x; 
    float world_acc_y;
    
    // --- 状态标志 ---
    uint8_t is_calibrated; 
} IMU_Car_Data_t;

extern IMU_Car_Data_t imu_car_data;

// ================= 函数声明 =================
void IMU_Car_Init(void);
void IMU_Car_Update_Loop(void);

#endif