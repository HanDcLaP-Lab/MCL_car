#ifndef _IMU_CAR_RC_H
#define _IMU_CAR_RC_H

// ================= 坐标系映射宏定义 =================
// 目标: 车体坐标系 (X前, Y右, Z下) - 符合右手定则
// 传感器安装: X向前, Y向右, Z向下

// 陀螺仪映射 (机体角速度)
// 目标Z向下，顺时针旋转为正。传感器Z向下，顺时针旋转读数为正，故GZ不取反。
#define IMU_MAP_GX(x, y, z)  (x)
#define IMU_MAP_GY(x, y, z)  (y)
#define IMU_MAP_GZ(x, y, z)  (z)

// 加速度计映射 (映射到重力向量方向，即 -1 * 机体加速度)
// Mahony算法要求: 静态平放时，Z轴分量应为正(指向地心，即+1g)
// 传感器Z向下，平放读数为-1g(支撑力向上)，故需取反适配目标系(+1g)
#define IMU_MAP_AX(x, y, z)  (x)
#define IMU_MAP_AY(x, y, z)  (y)
#define IMU_MAP_AZ(x, y, z)  (-z)

#ifndef PI
#define PI 3.1415926535f

// ================= 核心结构体定义 =================
typedef struct {
    // --- 姿态角 (单位: 度) ---
    float roll;
    float pitch;
    float yaw;          // 范围 -180 ~ +180

    // --- 扩展控制数据 ---
    float yaw_total;    // 范围 -inf ~ +inf (连续累计角度，用于多圈控制)
    float yaw_rate;     // Z轴角速度 (度/秒)，用于 PID D项，极重要！
    
    // --- 状态标志 ---
    uint8_t is_calibrated; 
} IMU_Car_Data_t;

extern IMU_Car_Data_t imu_car_data;

// ================= 函数声明 =================
void IMU_Car_RC_Init(void);
void IMU_Car_RC_Update_Loop(void);

#endif