#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include "arm_math.h"
typedef struct {
    float x;  // 状态变量（估计的速度/脉冲数）
    float p;  // 估计协方差
    float q;  // 过程噪声协方差（系统模型的不确定性）
    float r;  // 测量噪声协方差（传感器噪声）
    float k;  // 卡尔曼增益
} KalmanFilter1;

void Kalman_Init(KalmanFilter1* kf, float q, float r, float initial_value);
float Kalman_Update(KalmanFilter1* kf, float measurement) ;


// =====================================================================
// 麦克纳姆轮底盘扩展卡尔曼滤波器 (EKF) 定义
// =====================================================================
#define EKF_STATE_DIM 5  // 状态向量维度
#define EKF_OBS_DIM 2    // 观测向量维度

typedef struct {
    // 状态向量: [X_world, Y_world, Theta_world, V_x_body, V_y_body]^T
    float X_data[EKF_STATE_DIM];
    
    // 协方差矩阵 P (5x5)
    float P_data[EKF_STATE_DIM * EKF_STATE_DIM];
    
    // 过程噪声协方差矩阵 Q (5x5)
    float Q_data[EKF_STATE_DIM * EKF_STATE_DIM];
    
    // 观测噪声协方差矩阵 R (2x2) 以及其默认值
    float R_data[EKF_OBS_DIM * EKF_OBS_DIM];
    float R_default_data[EKF_OBS_DIM * EKF_OBS_DIM];

    // CMSIS-DSP 矩阵实例
    arm_matrix_instance_f32 X;
    arm_matrix_instance_f32 P;
    arm_matrix_instance_f32 Q;
    arm_matrix_instance_f32 R;

    // 打滑判定阈值
    float slip_threshold;
} Mecanum_EKF_t; 

extern Mecanum_EKF_t chassis_ekf;

void EKF_Init(Mecanum_EKF_t *ekf, float slip_thresh);
void EKF_Step(Mecanum_EKF_t *ekf, float ax, float ay, float omega, 
              float v1, float v2, float v3, float v4, float dt);


#endif