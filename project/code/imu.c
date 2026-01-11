#include "imu.h"

// ================= 全局变量定义 =================
IMU_Data_t imu_data = {0}; // 实例化结构体

// 内部算法变量
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差

// 校准相关
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
static uint16_t calib_cnt = 0;

extern uint32_t pit0_cnt;
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

// 核心姿态解算 (Mahony)
static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 计算加速度模长
    float accel_magnitude = sqrtf(ax*ax + ay*ay + az*az);

    // 2. 角度转弧度
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    // 3. 加速度归一化
    if (accel_magnitude < 0.1f) return; // 防止除0
    float inv_norm = 1.0f / accel_magnitude;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    // 4. 估计重力方向
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. 误差计算 (叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 6. 积分误差
    exInt += ex * KI * DT;
    eyInt += ey * KI * DT;
    ezInt += ez * KI * DT;

    // 7. 修正角速度
    gx += KP * ex + exInt;
    gy += KP * ey + eyInt;
    gz += KP * ez + ezInt;

    // 8. 四元数更新
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * DT);
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * DT);
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * DT);
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * DT);

    // 9. 四元数归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

// 惯性导航更新 (位置/速度)
static void Navigation_Update(float ax, float ay, float az) {
    // 1. 预计算四元数乘积
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    // 2. 将机体加速度旋转到世界坐标系
    // world_ax/ay/az 对应 北/东/地
    float w_ax = (1 - 2*(q2q2 + q3q3))*ax + 2*(q1q2 - q0q3)*ay + 2*(q1q3 + q0q2)*az;
    float w_ay = 2*(q1q2 + q0q3)*ax + (1 - 2*(q1q1 + q3q3))*ay + 2*(q2q3 - q0q1)*az;
    float w_az = 2*(q1q3 - q0q2)*ax + 2*(q2q3 + q0q1)*ay + (1 - 2*(q1q1 + q2q2))*az;

    // 3. 去除重力 (假设 Z 轴向下为正 9.8)
    w_az = w_az - GRAVITY_MSS;

    // 4. 零速修正 (死区)
    if(fabs(w_ax)<0.05)w_ax=0;
    if(fabs(w_ay)<0.05)w_ay=0;
    w_ax = Kalman_Update(&K_w_ax,w_ax);
    w_ay = Kalman_Update(&K_w_ay,w_ay);
    if (fabsf(w_az) < 0.2f) w_az = 0;

    // 更新到结构体
    imu_data.world_ax = w_ax;
    imu_data.world_ay = w_ay;
    imu_data.world_az = w_az;

    // ================== 核心：Z轴二阶互补滤波 ==================
    // 统一单位到 cm 和 cm/s，因为飞控 PID 通常用 cm
    float acc_up_cms2 = w_az * 100.0f; 
    
    // A. 预测步骤 (Prediction) - 纯积分
    // h = h + v*dt + 0.5*a*dt^2
    imu_data.z += imu_data.vz * DT + 0.5f * acc_up_cms2 * DT * DT;
    // v = v + a*dt
    imu_data.vz += acc_up_cms2 * DT;

    // B. 测量修正步骤 (Correction)
    // 只有当 ToF 数据有效时才修正 (假设范围 1cm - 200cm)
    if (imu_data.tof_z > 10 && imu_data.tof_z < 2000 && pit0_cnt%20 == 0) {
        // 1. 计算 ToF 投影高度 (几何修正)
        float rad_roll = imu_data.roll * (PI / 180.0f);
        float rad_pitch = imu_data.pitch * (PI / 180.0f);
        // 这里的 cos 修正非常重要，防止倾斜时高度数值虚高
        float kc = fabsf(cosf(rad_roll) * cosf(rad_pitch));
        float tof_height_cm = (imu_data.tof_z / 10.0f) * kc;
        float tof_height_speed_cms = (imu_data.tof_vz * 5.0f) * kc;
        // 2. 计算估计误差
        float z_error = tof_height_cm - imu_data.z;

        // 3. 修正位置 (P控制)
        imu_data.z += z_error * Z_CORRECT_POS_GAIN;

        // 4. 修正速度 (I控制，实际上是修正了速度的漂移)
        // 误差 * 增益 * DT -> 加到速度上
        //extern uint32_t pit0_cnt;
        //if(pit0_cnt%1000 == 0)printf("%.1f %.1f",tof_height_speed_cms,imu_data.vz);
        float vz_error = tof_height_speed_cms - imu_data.vz;
        imu_data.vz += vz_error * Z_CORRECT_VEL_GAIN;
    }

    // 5. 速度积分 (带阻尼)
    imu_data.vx += w_ax * DT *100.0f;
    imu_data.vy += w_ay * DT *100.0f;
    //if(pit0_cnt%20==0)printf(" %.2f,%.2f ",w_ax,w_ay);
    
    // 6. 位置积分 (cm)
    imu_data.x += imu_data.vx * DT * 100.0f;
    imu_data.y += imu_data.vy * DT * 100.0f;
}

// ================= 对外接口函数 =================
void IMU_Update_Loop(float tof_height_mm) {
    // 1. 获取并转换原始数据 (使用库宏 + 9.8系数)
    float raw_gx = imu660ra_gyro_transition(imu660ra_gyro_x);
    float raw_gy = imu660ra_gyro_transition(imu660ra_gyro_y);
    float raw_gz = imu660ra_gyro_transition(imu660ra_gyro_z);

    float raw_ax = imu660ra_acc_transition(imu660ra_acc_x) * GRAVITY_MSS;
    float raw_ay = imu660ra_acc_transition(imu660ra_acc_y) * GRAVITY_MSS;
    float raw_az = imu660ra_acc_transition(imu660ra_acc_z) * GRAVITY_MSS;

    // 2. 上电自动校准 (前2000次)
    if (imu_data.is_calibrated == 0) {
        calib_cnt++;
        offset_gx += raw_gx;
        offset_gy += raw_gy;
        offset_gz += raw_gz;
        
        if (calib_cnt >= 2000) {
            offset_gx /= 2000.0f;
            offset_gy /= 2000.0f;
            offset_gz /= 2000.0f;
            imu_data.is_calibrated = 1;
        }
        return; // 校准期间不更新姿态
    }

    // 3. 减去零偏
    raw_gx -= offset_gx;
    raw_gy -= offset_gy;
    raw_gz -= offset_gz;

    // 4. 坐标轴映射 (黄金公式：竖装，X上，Y前)
    float map_gx, map_gy, map_gz;
    float map_ax, map_ay, map_az;

    // 加速度映射
    map_ax = raw_ay;   // 前
    map_ay = -raw_az;  // 左
    map_az = -raw_ax;  // 下 (重力)

    // 陀螺仪映射
    map_gx = raw_gy;  // Roll
    map_gy = -raw_gz;  // Pitch
    map_gz = -raw_gx;  // Yaw


    // 5. 运行算法
    Mahony_Update(map_gx, map_gy, map_gz, map_ax, map_ay, map_az);
    
    // 6. 更新欧拉角到结构体
    // Roll
    imu_data.roll = -atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f + 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    // Pitch
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1) imu_data.pitch = copysignf(90.0f, sinp);
    else imu_data.pitch = asinf(sinp) * 180.0f / PI;
    // Yaw
    imu_data.yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

    // 7. 更新惯导位置
    Navigation_Update(map_ax, map_ay, map_az);
}