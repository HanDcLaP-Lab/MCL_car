#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

typedef struct {
    float x;  // 状态变量（估计的速度/脉冲数）
    float p;  // 估计协方差
    float q;  // 过程噪声协方差（系统模型的不确定性）
    float r;  // 测量噪声协方差（传感器噪声）
    float k;  // 卡尔曼增益
} KalmanFilter1;

void Kalman_Init(KalmanFilter1* kf, float q, float r, float initial_value);
float Kalman_Update(KalmanFilter1* kf, float measurement) ;
extern KalmanFilter1 K_w;
#endif