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

#define IMU_CAR_DT                    0.001f  // PIT_CH0 控制周期 1ms
#define IMU_CAR_CALIB_DISCARD_SAMPLES 500u    // 上电后先丢弃约 500ms IMU 启动瞬态
#define IMU_CAR_CALIB_SAMPLES         2000u   // 原始陀螺零偏校准时长约 2000ms
#define IMU_CAR_GYRO_DEADBAND         0.2f    // deg/s, 抑制静止 yaw 漂移
#define IMU_CAR_MAHONY_KP             0.93f   // roll/pitch 互补修正比例增益，移植自无人机
#define IMU_CAR_MAHONY_KI             0.0015f // roll/pitch 互补修正积分增益，运动时会降权
#define IMU_CAR_ACC_FULL_TRUST_ERR_G  0.04f   // 加速度模长偏离 1g 小于此值时完全信任
#define IMU_CAR_ACC_REJECT_ERR_G      0.08f   // 加速度模长偏离 1g 大于此值时停止加速度修正


// ================= 核心结构体定义 =================
typedef struct {
    // --- 姿态角 (单位: 度) ---
    // roll/pitch 来自 Mahony 姿态融合，会参与 yaw_rate 欧拉角投影。
    // yaw/yaw_total 仍只由陀螺仪积分，不接受加速度计直接修正。
    float roll;
    float pitch;
    float yaw;          // 范围 -180 ~ +180

    // --- 扩展控制数据 ---
    float yaw_total;    // 范围 -inf ~ +inf (连续累计角度，用于多圈控制)
    float yaw_rate;     // Z轴角速度 (度/秒)，用于 PID D项，极重要！
    
    // --- 状态标志 ---
    uint8_t is_calibrated; 
} IMU_Car_RC_Data_t;

extern IMU_Car_RC_Data_t imu_car_rc_data;

// ================= 函数声明 =================
void IMU_Car_RC_Init(void);
void IMU_Car_RC_Update_Loop(void);

#endif
