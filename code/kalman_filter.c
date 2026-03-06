// 初始化函数
#include "kalman_filter.h"
#include "zf_common_headfile.h"

KalmanFilter1 K_w;  // 卡尔曼定义
void Kalman_Init(KalmanFilter1* kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1;
    kf->k = 0;
}


// 卡尔曼滤波更新函数
float Kalman_Update(KalmanFilter1* kf, float measurement) {
    kf->p = kf->p + kf->q;                          // 预测
    kf->k = kf->p / (kf->p + kf->r);                // 更新卡尔曼增益
    kf->x = kf->x + kf->k * (measurement - kf->x);  // 更新估计值
    kf->p = (1 - kf->k) * kf->p;                    // 更新误差协方差
    return kf->x;
}