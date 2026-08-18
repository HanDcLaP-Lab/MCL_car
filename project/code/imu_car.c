#include "imu_car.h"
#include <math.h>

IMU_Car_Data_t imu_car_data = {0};

static double sum_gx = 0.0;
static double sum_gy = 0.0;
static double sum_gz = 0.0;
static double sum_ax = 0.0;
static double sum_ay = 0.0;
static double sum_az = 0.0;
static float offset_gx = 0.0f;
static float offset_gy = 0.0f;
static float offset_gz = 0.0f;
static uint16_t calib_discard_cnt = 0;
static uint16_t calib_cnt = 0;

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float exInt = 0.0f;
static float eyInt = 0.0f;

static KalmanFilter1 k_acc_x;
static KalmanFilter1 k_acc_y;
static KalmanFilter1 k_acc_z;
static KalmanFilter1 k_gyro_x;
static KalmanFilter1 k_gyro_y;
static KalmanFilter1 k_gyro_z;

static float Wrap_Angle_180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void IMU_Car_Reset_Attitude_State(void) {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0f;
    eyInt = 0.0f;
}

static void IMU_Car_Init_Filters(void) {
    Kalman_Init(&k_acc_x, 0.001f, 0.05f, 0.0f);
    Kalman_Init(&k_acc_y, 0.001f, 0.05f, 0.0f);
    Kalman_Init(&k_acc_z, 0.001f, 0.05f, 0.0f);
    Kalman_Init(&k_gyro_x, 1e-3f, 1e-3f, 0.0f);
    Kalman_Init(&k_gyro_y, 1e-3f, 1e-3f, 0.0f);
    Kalman_Init(&k_gyro_z, 1e-3f, 1e-3f, 0.0f);
}

static void IMU_Car_Init_Attitude_From_Accel(float avg_ax, float avg_ay, float avg_az) {
    float init_ax = IMU_MAP_AX(avg_ax, avg_ay, avg_az);
    float init_ay = IMU_MAP_AY(avg_ax, avg_ay, avg_az);
    float init_az = IMU_MAP_AZ(avg_ax, avg_ay, avg_az);

    float init_roll = atan2f(init_ay, init_az);
    float init_pitch = atan2f(-init_ax, sqrtf(init_ay * init_ay + init_az * init_az));
    float init_yaw = 0.0f;

    float cy = cosf(init_yaw * 0.5f);
    float sy = sinf(init_yaw * 0.5f);
    float cp = cosf(init_pitch * 0.5f);
    float sp = sinf(init_pitch * 0.5f);
    float cr = cosf(init_roll * 0.5f);
    float sr = sinf(init_roll * 0.5f);

    q0 = cy * cp * cr + sy * sp * sr;
    q1 = cy * cp * sr - sy * sp * cr;
    q2 = cy * sp * cr + sy * cp * sr;
    q3 = sy * cp * cr - cy * sp * sr;

    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.001f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    } else {
        IMU_Car_Reset_Attitude_State();
    }

    exInt = 0.0f;
    eyInt = 0.0f;
    imu_car_data.roll = init_roll * 180.0f / PI;
    imu_car_data.pitch = init_pitch * 180.0f / PI;
}

static void IMU_Car_Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az) {
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm < 0.1f || acc_norm != acc_norm) {
        return;
    }

    float acc_weight = 1.0f;
    float error_magnitude = fabsf(acc_norm - 1.0f);
    if (error_magnitude > IMU_CAR_ACC_REJECT_ERR_G) {
        acc_weight = 0.0f;
    } else if (error_magnitude > IMU_CAR_ACC_FULL_TRUST_ERR_G) {
        acc_weight = 1.0f - (error_magnitude - IMU_CAR_ACC_FULL_TRUST_ERR_G)
                   / (IMU_CAR_ACC_REJECT_ERR_G - IMU_CAR_ACC_FULL_TRUST_ERR_G);
    }

    float inv_norm = 1.0f / acc_norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    // 运动加速度的模长可能仍接近1g；方向创新用于阻止其污染roll/pitch。
    float direction_error = sqrtf(ex * ex + ey * ey + ez * ez);
    float direction_weight = 1.0f;
    if (direction_error >= IMU_CAR_ACC_DIR_REJECT_ERROR) {
        direction_weight = 0.0f;
    } else if (direction_error > IMU_CAR_ACC_DIR_FULL_TRUST_ERROR) {
        direction_weight = (IMU_CAR_ACC_DIR_REJECT_ERROR - direction_error)
                         / (IMU_CAR_ACC_DIR_REJECT_ERROR - IMU_CAR_ACC_DIR_FULL_TRUST_ERROR);
    }
    if (direction_weight < acc_weight) acc_weight = direction_weight;

    if (acc_weight > 0.1f) {
        exInt += ex * IMU_CAR_MAHONY_KI * IMU_CAR_DT * acc_weight;
        eyInt += ey * IMU_CAR_MAHONY_KI * IMU_CAR_DT * acc_weight;
    }

    gx = (gx * (PI / 180.0f)) + IMU_CAR_MAHONY_KP * acc_weight * ex + exInt;
    gy = (gy * (PI / 180.0f)) + IMU_CAR_MAHONY_KP * acc_weight * ey + eyInt;
    gz *= (PI / 180.0f);

    float q0_last = q0;
    float q1_last = q1;
    float q2_last = q2;
    float q3_last = q3;

    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * (0.5f * IMU_CAR_DT);
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * (0.5f * IMU_CAR_DT);
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * (0.5f * IMU_CAR_DT);
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * (0.5f * IMU_CAR_DT);

    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.001f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
}

static uint8_t imu_type_is_660ra = 0; // 0: 660RC, 1: 660RA

void IMU_Car_Init(void){
    sum_gx = 0.0;
    sum_gy = 0.0;
    sum_gz = 0.0;
    sum_ax = 0.0;
    sum_ay = 0.0;
    sum_az = 0.0;
    offset_gx = 0.0f;
    offset_gy = 0.0f;
    offset_gz = 0.0f;
    calib_discard_cnt = 0;
    calib_cnt = 0;
    imu_car_data.is_calibrated = 0;
    IMU_Car_Reset_Attitude_State();
    IMU_Car_Init_Filters();

    uint8_t retry = 0;
    while(retry++ < 10)
    {
        if(imu660rc_init(IMU660RC_QUARTERNION_DISABLE) == 0)                    // 优先尝试 660RC
        {
            imu_type_is_660ra = 0;
            printf("\r\n imu660rc init ok.");
            break;
        }
        else if(imu660ra_init() == 0)                                           // 备选尝试 660RA
        {
            imu_type_is_660ra = 1;
            printf("\r\n imu660ra init ok.");
            break;
        }
        else
        {
            printf("\r\n imu init retry %d...", retry);
            system_delay_ms(20);
        }
    }
}

void IMU_Car_Update_Loop(void){
    float raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az;

    if (imu_type_is_660ra) {
        imu660ra_get_acc();
        imu660ra_get_gyro();
        if (imu660ra_acc_x == 0 && imu660ra_acc_y == 0 && imu660ra_acc_z == 0
            && imu660ra_gyro_x == 0 && imu660ra_gyro_y == 0 && imu660ra_gyro_z == 0) {
            if (++calib_discard_cnt > 3000) imu_car_data.is_calibrated = 1; // 超时强行放行，防死锁
            return;
        }
        raw_gx = imu660ra_gyro_transition(imu660ra_gyro_x);
        raw_gy = imu660ra_gyro_transition(imu660ra_gyro_y);
        raw_gz = imu660ra_gyro_transition(imu660ra_gyro_z);
        raw_ax = imu660ra_acc_transition(imu660ra_acc_x);
        raw_ay = imu660ra_acc_transition(imu660ra_acc_y);
        raw_az = imu660ra_acc_transition(imu660ra_acc_z);
    } else {
        imu660rc_get_acc();
        imu660rc_get_gyro();
        if (imu660rc_acc_x == 0 && imu660rc_acc_y == 0 && imu660rc_acc_z == 0
            && imu660rc_gyro_x == 0 && imu660rc_gyro_y == 0 && imu660rc_gyro_z == 0) {
            if (++calib_discard_cnt > 3000) imu_car_data.is_calibrated = 1; // 超时强行放行，防死锁
            return;
        }
        raw_gx = imu660rc_gyro_transition(imu660rc_gyro_x);
        raw_gy = imu660rc_gyro_transition(imu660rc_gyro_y);
        raw_gz = imu660rc_gyro_transition(imu660rc_gyro_z);
        raw_ax = imu660rc_acc_transition(imu660rc_acc_x);
        raw_ay = imu660rc_acc_transition(imu660rc_acc_y);
        raw_az = imu660rc_acc_transition(imu660rc_acc_z);
    }

    raw_ax = Kalman_Update(&k_acc_x, raw_ax);
    raw_ay = Kalman_Update(&k_acc_y, raw_ay);
    raw_az = Kalman_Update(&k_acc_z, raw_az);
    raw_gx = Kalman_Update(&k_gyro_x, raw_gx);
    raw_gy = Kalman_Update(&k_gyro_y, raw_gy);
    raw_gz = Kalman_Update(&k_gyro_z, raw_gz);

    if (imu_car_data.is_calibrated == 0) {
        if (calib_discard_cnt < IMU_CAR_CALIB_DISCARD_SAMPLES) {
            calib_discard_cnt++;
            return;
        }

        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;
        sum_ax += raw_ax;
        sum_ay += raw_ay;
        sum_az += raw_az;

        if (++calib_cnt >= IMU_CAR_CALIB_SAMPLES) {
            offset_gx = (float)(sum_gx / (double)IMU_CAR_CALIB_SAMPLES);
            offset_gy = (float)(sum_gy / (double)IMU_CAR_CALIB_SAMPLES);
            offset_gz = (float)(sum_gz / (double)IMU_CAR_CALIB_SAMPLES);
            IMU_Car_Init_Attitude_From_Accel(
                (float)(sum_ax / (double)IMU_CAR_CALIB_SAMPLES),
                (float)(sum_ay / (double)IMU_CAR_CALIB_SAMPLES),
                (float)(sum_az / (double)IMU_CAR_CALIB_SAMPLES));

            imu_car_data.yaw = 0.0f;
            imu_car_data.yaw_total = 0.0f;
            imu_car_data.yaw_rate = 0.0f;
            imu_car_data.is_calibrated = 1;
        }
        return;
    }

    raw_gx -= offset_gx;
    raw_gy -= offset_gy;
    raw_gz -= offset_gz;

    float map_ax = IMU_MAP_AX(raw_ax, raw_ay, raw_az);
    float map_ay = IMU_MAP_AY(raw_ax, raw_ay, raw_az);
    float map_az = IMU_MAP_AZ(raw_ax, raw_ay, raw_az);

    float map_gx = IMU_MAP_GX(raw_gx, raw_gy, raw_gz);
    float map_gy = IMU_MAP_GY(raw_gx, raw_gy, raw_gz);
    float map_gz = IMU_MAP_GZ(raw_gx, raw_gy, raw_gz);

    if (fabsf(map_gz) < IMU_CAR_GYRO_DEADBAND) {
        map_gz = 0.0f;
    }

    // roll/pitch 来自 Mahony 融合；加速度模长异常时自动降权，避免坡面运动把线加速度当成姿态。
    IMU_Car_Mahony_Update(map_gx, map_gy, map_gz, map_ax, map_ay, map_az);
    imu_car_data.roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        imu_car_data.pitch = (sinp >= 0.0f) ? 90.0f : -90.0f;
    } else {
        imu_car_data.pitch = asinf(sinp) * 180.0f / PI;
    }

    float roll_rad = imu_car_data.roll * (PI / 180.0f);
    float pitch_rad = imu_car_data.pitch * (PI / 180.0f);
    float cos_pitch = cosf(pitch_rad);
    if (fabsf(cos_pitch) < 0.1f) {
        cos_pitch = (cos_pitch >= 0.0f) ? 0.1f : -0.1f;
    }

    float yaw_rate = (map_gy * sinf(roll_rad) + map_gz * cosf(roll_rad)) / cos_pitch;
    if (fabsf(yaw_rate) < IMU_CAR_GYRO_DEADBAND) {
        yaw_rate = 0.0f;
    }

    imu_car_data.yaw_rate = yaw_rate;
    imu_car_data.yaw_total += yaw_rate * IMU_CAR_DT;
    imu_car_data.yaw = Wrap_Angle_180(imu_car_data.yaw_total);
}
