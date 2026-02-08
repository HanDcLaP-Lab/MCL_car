#include "imu_car.h"

// ================= 全局变量定义 =================
IMU_Car_Data_t imu_car_data = {0}; 

// 内部算法变量
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差

// 校准相关变量
static float offset_gx = 0, offset_gy = 0, offset_gz = 0;
static float sum_gx = 0, sum_gy = 0, sum_gz = 0;
static float sum_ax = 0, sum_ay = 0, sum_az = 0;
static uint16_t calib_cnt = 0;

// 辅助函数：平方根倒数
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void IMU_Car_Init(void){
    // 初始化硬件 (假设延用原底层)
    while(1)
    {
        if(imu660ra_init()) 
        {
           printf("IMU Init Error\n");   
        }
        else
        {
           break;
        } 
        system_delay_ms(100);                                       
    }
}

// Mahony 姿态解算核心 (保留原版逻辑)
static void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 计算加速度模长
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    // 2. 角度转弧度
    gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

    // ==============================================================================
    // 【动态权重逻辑】
    // 麦轮小车急停时惯性大，会产生虚假加速度。
    // 当加速度模长偏离重力太大时，降低加速度计权重，防止姿态被"甩"歪。
    // ==============================================================================
    float acc_weight = 1.0f;
    float error_magnitude = fabsf(acc_norm - GRAVITY_MSS); 

    if (error_magnitude > 2.0f) { // 偏差 > 0.2g (可根据小车爆发力调整)
        acc_weight = 0.0f; 
    } else if (error_magnitude > 0.5f) {
        acc_weight = 1.0f - (error_magnitude - 0.5f) / (1.5f);
    }

    // 3. 加速度归一化
    if (acc_norm < 0.1f) return; 
    float inv_norm = 1.0f / acc_norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    // 4. 估计重力方向 (基于四元数)
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. 误差计算 (叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 6. 积分误差补偿
    if (acc_weight > 0.05f) { 
        exInt += ex * IMU_KI * IMU_DT * acc_weight;
        eyInt += ey * IMU_KI * IMU_DT * acc_weight;
    }
    
    // 7. 修正角速度
    gx += IMU_KP * acc_weight * ex + exInt;
    gy += IMU_KP * acc_weight * ey + eyInt;
    
    // Z轴(Yaw)不接受加速度计修正，只靠陀螺仪积分
    gz += 0; 

    // 8. 四元数更新
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * IMU_DT);
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * IMU_DT);
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * IMU_DT);
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * IMU_DT);

    // 9. 四元数归一化
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

// 仅用于计算世界坐标系加速度 (用于撞击检测)，不进行位移积分
static void World_Accel_Calc(float ax, float ay, float az) {
    float q0q1 = q0 * q1, q0q2 = q0 * q2, q0q3 = q0 * q3;
    float q1q1 = q1 * q1, q1q2 = q1 * q2, q1q3 = q1 * q3;
    float q2q2 = q2 * q2, q2q3 = q2 * q3, q3q3 = q3 * q3;

    // 旋转到世界坐标系
    float w_ax = (1 - 2*(q2q2 + q3q3))*ax + 2*(q1q2 - q0q3)*ay + 2*(q1q3 + q0q2)*az;
    float w_ay = 2*(q1q2 + q0q3)*ax + (1 - 2*(q1q1 + q3q3))*ay + 2*(q2q3 - q0q1)*az;
    
    // 简单的死区过滤
    if(fabsf(w_ax) < 0.1f) w_ax = 0; 
    if(fabsf(w_ay) < 0.1f) w_ay = 0;

    imu_car_data.world_acc_x = w_ax;
    imu_car_data.world_acc_y = w_ay;
}

// ================= 对外接口函数 =================

void IMU_Car_Update_Loop(void) {
    
    // 1. 获取硬件数据
    imu660ra_get_acc();
    imu660ra_get_gyro();
    
    // 原始数据转换
    float raw_gx = imu660ra_gyro_transition(imu660ra_gyro_x);
    float raw_gy = imu660ra_gyro_transition(imu660ra_gyro_y);
    float raw_gz = imu660ra_gyro_transition(imu660ra_gyro_z);

    float raw_ax = imu660ra_acc_transition(imu660ra_acc_x) * GRAVITY_MSS;
    float raw_ay = imu660ra_acc_transition(imu660ra_acc_y) * GRAVITY_MSS;
    float raw_az = imu660ra_acc_transition(imu660ra_acc_z) * GRAVITY_MSS;

    // ================= 校准阶段 (包含安装误差修正) =================
    if (imu_car_data.is_calibrated == 0) {
        calib_cnt++;
        
        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;
        sum_ax += raw_ax;
        sum_ay += raw_ay;
        sum_az += raw_az;
        
        // 持续 2.5秒 (假设1ms周期)
        if (calib_cnt >= 2500) {
            // 1. 计算陀螺仪零偏
            offset_gx = (float)(sum_gx / 2500.0);
            offset_gy = (float)(sum_gy / 2500.0);
            offset_gz = (float)(sum_gz / 2500.0);

            // 2. 计算平均重力向量 (用于修正安装倾角)
            float avg_ax = (float)(sum_ax / 2500.0);
            float avg_ay = (float)(sum_ay / 2500.0);
            float avg_az = (float)(sum_az / 2500.0);

            // 将原始加速度映射到算法坐标系
            float init_ax = IMU_MAP_AX(avg_ax, avg_ay, avg_az);
            float init_ay = IMU_MAP_AY(avg_ax, avg_ay, avg_az);
            float init_az = IMU_MAP_AZ(avg_ax, avg_ay, avg_az);

            // 3. 核心步骤：根据重力向量初始化四元数
            // 这步操作会将当前的"物理倾角"在算法中视为"水平(0度)"
            // 从而自动补偿传感器的安装误差
            float init_roll  = atan2f(init_ay, init_az);
            float init_pitch = atan2f(-init_ax, sqrtf(init_ay*init_ay + init_az*init_az));
            float init_yaw   = 0.0f; // 初始 Yaw 默认为 0

            float c1 = cosf(init_yaw / 2); float s1 = sinf(init_yaw / 2);
            float c2 = cosf(init_pitch / 2); float s2 = sinf(init_pitch / 2);
            float c3 = cosf(init_roll / 2); float s3 = sinf(init_roll / 2);

            q0 = c1*c2*c3 + s1*s2*s3;
            q1 = c1*c2*s3 - s1*s2*c3;
            q2 = c1*s2*c3 + s1*c2*s3;
            q3 = s1*c2*c3 - c1*s2*s3;
            
            // 归一化四元数
            float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
            q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

            imu_car_data.is_calibrated = 1;
        }
        return; 
    }

    // ================= 正常运行阶段 =================

    // 1. 去除陀螺仪零偏
    raw_gx -= offset_gx;
    raw_gy -= offset_gy;
    raw_gz -= offset_gz;

    // 2. 坐标系映射
    float map_ax = IMU_MAP_AX(raw_ax, raw_ay, raw_az);
    float map_ay = IMU_MAP_AY(raw_ax, raw_ay, raw_az);
    float map_az = IMU_MAP_AZ(raw_ax, raw_ay, raw_az);

    float map_gx = IMU_MAP_GX(raw_gx, raw_gy, raw_gz);
    float map_gy = IMU_MAP_GY(raw_gx, raw_gy, raw_gz);
    float map_gz = IMU_MAP_GZ(raw_gx, raw_gy, raw_gz);

    // 3. 死区处理 (防止静止时 Yaw 缓慢漂移)
    if (fabsf(map_gz) < VALID_G_MIN) map_gz = 0;

    // 输出实时角速度 (重要：用于PID控制)
    imu_car_data.yaw_rate = map_gz;

    // 4. Mahony 姿态解算
    Mahony_Update(map_gx, map_gy, map_gz, map_ax, map_ay, map_az);
    
    // 5. 欧拉角转换 (四元数 -> 欧拉角)
    // Roll
    imu_car_data.roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    
    // Pitch
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1) imu_car_data.pitch = copysignf(90.0f, sinp);
    else imu_car_data.pitch = asinf(sinp) * 180.0f / PI;

    // Yaw (临时变量)
    float new_yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

    // ================= 6. 多圈角度累积处理 =================
    // 解决 -180 到 180 跳变问题
    float yaw_diff = new_yaw - imu_car_data.yaw;
    
    // 处理过零点跳变
    if (yaw_diff < -180.0f) yaw_diff += 360.0f;
    else if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
    
    imu_car_data.yaw = new_yaw;         // 保持 -180~180 输出
    imu_car_data.yaw_total += yaw_diff; // 累积角度，如 720.5度

    // 7. 计算世界坐标系加速度 (可选)
    World_Accel_Calc(map_ax, map_ay, map_az);
}