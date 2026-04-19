#ifndef _IMU_CAR_RC_H
#define _IMU_CAR_RC_H

#include "zf_common_headfile.h" 


// ================= 坐标系映射宏定义 =================
// 目标: 车体坐标系 (X前, Y右, Z下) - 符合右手定则
// 现在的传感器安装: X向右, Y向后, Z向下

// 陀螺仪映射 (机体角速度)
// 车体X(前) = 传感器-Y(因为Y向后)
#define IMU_MAP_GX(x, y, z)  (-y)
// 车体Y(右) = 传感器X(因为X向右)
#define IMU_MAP_GY(x, y, z)  (x)
// 车体Z(下) = 传感器Z(因为Z向下)
#define IMU_MAP_GZ(x, y, z)  (z)

// 加速度计映射 (映射到重力向量方向，即 -1 * 机体加速度)
// 保持与陀螺仪相同的物理XY映射，Z轴保留取反(适配Mahony算法需要的+1g特征)
#define IMU_MAP_AX(x, y, z)  (-y)
#define IMU_MAP_AY(x, y, z)  (x)
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
    
    float ax; //车体系加速度，向前x向右y，单位m/s2
    float ay;
    float az;
    // --- 状态标志 ---
    uint8_t is_calibrated; 
} IMU_Car_RC_Data_t;

extern IMU_Car_RC_Data_t imu_car_rc_data;

// ================= 函数声明 =================
void IMU_Car_RC_Init(void);
void IMU_Car_RC_Update_Loop(void);

#endif