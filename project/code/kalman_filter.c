// 初始化函数
#include "kalman_filter.h"
#include "zf_common_headfile.h"
#include "string.h"
#include "arm_math.h"

Mecanum_EKF_t chassis_ekf;

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

// =====================================================================
// 麦克纳姆轮底盘扩展卡尔曼滤波器 (EKF) 核心实现
// =====================================================================

/**
 * @brief 初始化 EKF 实例与协方差矩阵
 */
void EKF_Init(Mecanum_EKF_t *ekf, float slip_thresh) {
    ekf->slip_threshold = slip_thresh;
    
    // 清零底层数组
    memset(ekf->X_data, 0, sizeof(ekf->X_data));
    memset(ekf->P_data, 0, sizeof(ekf->P_data));
    memset(ekf->Q_data, 0, sizeof(ekf->Q_data));
    memset(ekf->R_data, 0, sizeof(ekf->R_data));
    memset(ekf->R_default_data, 0, sizeof(ekf->R_default_data));

    // 初始化 CMSIS-DSP 矩阵结构关联
    arm_mat_init_f32(&ekf->X, EKF_STATE_DIM, 1, ekf->X_data);
    arm_mat_init_f32(&ekf->P, EKF_STATE_DIM, EKF_STATE_DIM, ekf->P_data);
    arm_mat_init_f32(&ekf->Q, EKF_STATE_DIM, EKF_STATE_DIM, ekf->Q_data);
    arm_mat_init_f32(&ekf->R, EKF_OBS_DIM, EKF_OBS_DIM, ekf->R_data);

    // 设置 P, Q 的初始对角线元素
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        ekf->P_data[i * EKF_STATE_DIM + i] = 1.0f;    // 初始协方差较大，代表初始不确定
        ekf->Q_data[i * EKF_STATE_DIM + i] = 0.01f;   // 状态转移信任度，需调参
    }
    
    // R 矩阵默认值 (编码器解算观测方差)
    ekf->R_default_data[0] = 0.05f; // V_x 测量方差
    ekf->R_default_data[3] = 0.05f; // V_y 测量方差
    memcpy(ekf->R_data, ekf->R_default_data, sizeof(ekf->R_data));
}

/**
 * @brief 麦轮 EKF 核心步进迭代计算
 * @param v1 左前轮线速度 
 * @param v2 右前轮线速度
 * @param v3 左后轮线速度
 * @param v4 右后轮线速度
 */
void EKF_Step(Mecanum_EKF_t *ekf, float ax, float ay, float omega, 
              float v1, float v2, float v3, float v4, float dt) {
    // =========================================================
    // Step 1: 打滑检测与自适应协方差 (Adaptive R)
    // =========================================================
    // 根据麦轮正逆解特性：理想纯平移下，对角轮的速度应该严格一致
    float diff_A = fabsf(v1 - v4); 
    float diff_B = fabsf(v2 - v3); 
    
    if (fabsf(omega) < 0.05f && (diff_A > ekf->slip_threshold || diff_B > ekf->slip_threshold)) {
        // 检测到打滑，极速膨胀观测方差 (x100)，降低对轮式里程计的信任
        ekf->R_data[0] = ekf->R_default_data[0] * 100.0f;
        ekf->R_data[3] = ekf->R_default_data[3] * 100.0f;
    } else {
        // 恢复正常观测信任度
        ekf->R_data[0] = ekf->R_default_data[0];
        ekf->R_data[3] = ekf->R_default_data[3];
    }

    // =========================================================
    // Step 2: 状态预测 (Predict)
    // =========================================================
    float x     = ekf->X_data[0];
    float y     = ekf->X_data[1];
    float theta = ekf->X_data[2];
    float vx    = ekf->X_data[3];
    float vy    = ekf->X_data[4];

    // 调用 CMSIS 优化的三角函数
    float sin_theta = arm_sin_f32(theta);
    float cos_theta = arm_cos_f32(theta);

    // 1. 根据运动学非线性方程预测下一时刻状态 (Euler 积分)
    ekf->X_data[0] = x + (vx * cos_theta - vy * sin_theta) * dt;
    ekf->X_data[1] = y + (vx * sin_theta + vy * cos_theta) * dt;
    ekf->X_data[2] = theta + omega * dt;
    ekf->X_data[3] = vx + ax * dt;
    ekf->X_data[4] = vy + ay * dt;

    // 2. 构建雅可比矩阵 F (5x5)
    float F_f32[25] = {0};
    arm_matrix_instance_f32 F;
    arm_mat_init_f32(&F, EKF_STATE_DIM, EKF_STATE_DIM, F_f32);
    
    F_f32[0]  = 1.0f;  // dX/dX
    F_f32[2]  = (-vx * sin_theta - vy * cos_theta) * dt; // dX/dTheta
    F_f32[3]  = cos_theta * dt;  // dX/dVx
    F_f32[4]  = -sin_theta * dt; // dX/dVy
    
    F_f32[6]  = 1.0f;  // dY/dY
    F_f32[7]  = (vx * cos_theta - vy * sin_theta) * dt; // dY/dTheta
    F_f32[8]  = sin_theta * dt;  // dY/dVx
    F_f32[9]  = cos_theta * dt;  // dY/dVy
    
    F_f32[12] = 1.0f;  // dTheta/dTheta
    F_f32[18] = 1.0f;  // dVx/dVx
    F_f32[24] = 1.0f;  // dVy/dVy

    // 3. 预测协方差矩阵 P = F * P * F^T + Q
    float Ft_f32[25], FP_f32[25], FPFt_f32[25];
    arm_matrix_instance_f32 Ft, FP, FPFt;
    arm_mat_init_f32(&Ft, EKF_STATE_DIM, EKF_STATE_DIM, Ft_f32);
    arm_mat_init_f32(&FP, EKF_STATE_DIM, EKF_STATE_DIM, FP_f32);
    arm_mat_init_f32(&FPFt, EKF_STATE_DIM, EKF_STATE_DIM, FPFt_f32);

    arm_mat_trans_f32(&F, &Ft);
    arm_mat_mult_f32(&F, &ekf->P, &FP);
    arm_mat_mult_f32(&FP, &Ft, &FPFt);
    arm_mat_add_f32(&FPFt, &ekf->Q, &ekf->P); // 结果安全覆盖回 ekf->P

    // =========================================================
    // Step 3: 观测更新 (Update)
    // =========================================================
    // 1. 基于编码器解算观测值 Z (麦轮正向运动学公式)
    float Z_data[2];
    Z_data[0] = (v1 + v2 + v3 + v4) * 0.25f;  
    Z_data[1] = (v1 - v2 - v3 + v4) * 0.25f; 
    arm_matrix_instance_f32 Z;
    arm_mat_init_f32(&Z, EKF_OBS_DIM, 1, Z_data);

    // 2. 观测模型矩阵 H (提取状态里的 V_x_body, V_y_body)
    float H_f32[10] = {0};
    H_f32[3] = 1.0f; // 对应第4列 (V_x)
    H_f32[9] = 1.0f; // 对应第5列 (V_y)
    arm_matrix_instance_f32 H;
    arm_mat_init_f32(&H, EKF_OBS_DIM, EKF_STATE_DIM, H_f32);

    // 3. 计算新息 Y_res = Z - H * X
    float HX_data[2];
    arm_matrix_instance_f32 HX;
    arm_mat_init_f32(&HX, EKF_OBS_DIM, 1, HX_data);
    arm_mat_mult_f32(&H, &ekf->X, &HX);
    
    float Y_res_data[2];
    arm_matrix_instance_f32 Y_res;
    arm_mat_init_f32(&Y_res, EKF_OBS_DIM, 1, Y_res_data);
    arm_mat_sub_f32(&Z, &HX, &Y_res);

    // 4. 计算新息协方差 S = H * P * H^T + R
    float Ht_f32[10], PHt_f32[10], HPHt_f32[4], S_f32[4];
    arm_matrix_instance_f32 Ht, PHt, HPHt, S;
    arm_mat_init_f32(&Ht, EKF_STATE_DIM, EKF_OBS_DIM, Ht_f32);
    arm_mat_init_f32(&PHt, EKF_STATE_DIM, EKF_OBS_DIM, PHt_f32);
    arm_mat_init_f32(&HPHt, EKF_OBS_DIM, EKF_OBS_DIM, HPHt_f32);
    arm_mat_init_f32(&S, EKF_OBS_DIM, EKF_OBS_DIM, S_f32);

    arm_mat_trans_f32(&H, &Ht);
    arm_mat_mult_f32(&ekf->P, &Ht, &PHt); // 保存 PHt 供后面计算卡尔曼增益用
    arm_mat_mult_f32(&H, &PHt, &HPHt);
    arm_mat_add_f32(&HPHt, &ekf->R, &S);

    // 5. 【性能榨取】对 2x2 矩阵 S 求逆解析解
    float S_inv_f32[4];
    arm_matrix_instance_f32 S_inv;
    arm_mat_init_f32(&S_inv, EKF_OBS_DIM, EKF_OBS_DIM, S_inv_f32);
    
    float det = S_f32[0] * S_f32[3] - S_f32[1] * S_f32[2];
    if (fabsf(det) > 1e-8f) { // 规避除零风险
        float inv_det = 1.0f / det;
        S_inv_f32[0] =  S_f32[3] * inv_det;
        S_inv_f32[1] = -S_f32[1] * inv_det;
        S_inv_f32[2] = -S_f32[2] * inv_det;
        S_inv_f32[3] =  S_f32[0] * inv_det;
    } else {
        // 病态矩阵保护，退化为单位阵
        S_inv_f32[0] = 1.0f; S_inv_f32[1] = 0.0f;
        S_inv_f32[2] = 0.0f; S_inv_f32[3] = 1.0f;
    }

    // 6. 计算卡尔曼增益 K = P * H^T * S_inv (刚才存的 PHt 就用上了)
    float K_f32[10];
    arm_matrix_instance_f32 K;
    arm_mat_init_f32(&K, EKF_STATE_DIM, EKF_OBS_DIM, K_f32);
    arm_mat_mult_f32(&PHt, &S_inv, &K);

    // 7. 更新状态向量 X = X + K * Y_res
    float KY_data[5];
    arm_matrix_instance_f32 KY;
    arm_mat_init_f32(&KY, EKF_STATE_DIM, 1, KY_data);
    arm_mat_mult_f32(&K, &Y_res, &KY);
    arm_mat_add_f32(&ekf->X, &KY, &ekf->X);

    // 8. 更新协方差矩阵 P = (I - K * H) * P
    float KH_f32[25], I_f32[25] = {0}, I_KH_f32[25], P_new_f32[25];
    arm_matrix_instance_f32 KH, I, I_KH, P_new;
    arm_mat_init_f32(&KH, EKF_STATE_DIM, EKF_STATE_DIM, KH_f32);
    arm_mat_init_f32(&I, EKF_STATE_DIM, EKF_STATE_DIM, I_f32);
    arm_mat_init_f32(&I_KH, EKF_STATE_DIM, EKF_STATE_DIM, I_KH_f32);
    arm_mat_init_f32(&P_new, EKF_STATE_DIM, EKF_STATE_DIM, P_new_f32);
    
    // 构造单位矩阵 I
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        I_f32[i * EKF_STATE_DIM + i] = 1.0f; 
    }

    arm_mat_mult_f32(&K, &H, &KH);
    arm_mat_sub_f32(&I, &KH, &I_KH);
    arm_mat_mult_f32(&I_KH, &ekf->P, &P_new);

    // 覆盖回结构体原 P 矩阵
    memcpy(ekf->P_data, P_new_f32, sizeof(P_new_f32)); 

    // =========================================================
    // Step 4: 协方差矩阵保护 (强制对称化)
    // =========================================================
    // 算法：P = (P + P^T) / 2.0f
    float Pt_f32[25];
    arm_matrix_instance_f32 Pt;
    arm_mat_init_f32(&Pt, EKF_STATE_DIM, EKF_STATE_DIM, Pt_f32);

    arm_mat_trans_f32(&ekf->P, &Pt);            // 1. P^T
    arm_mat_add_f32(&ekf->P, &Pt, &P_new);      // 2. (P + P^T) -> 存到 P_new
    
    // 3. 结果除以2使用 CMSIS 标量乘法直接写回 P 数组，完全不消耗额外内存
    arm_scale_f32(P_new_f32, 0.5f, ekf->P_data, EKF_STATE_DIM * EKF_STATE_DIM); 

    // =========================================================
    // Step 5: 强制状态解耦与协方差限幅 (修复 X_data 卡死 BUG)
    // =========================================================
    // 原因：X, Y, Theta 是纯积分累加的不可观测状态。
    // 在单精度浮点 EKF 中，如果不加限制，其交叉协方差项会随时间无限累积，
    // 导致卡尔曼增益 K 爆炸，使得微小的观测噪声引发位置的巨大突变或飞出浮点精度上限从而卡死。
    
    for (int i = 0; i < 3; i++) {
        // 1. 限制不可观测状态的自身方差上限，防止加法截断
        int diag_idx = i * EKF_STATE_DIM + i;
        if (ekf->P_data[diag_idx] > 10.0f) {
            ekf->P_data[diag_idx] = 10.0f;
        }
        // 2. 清除位置/航向与速度之间的交叉协方差，强制关闭对位置的"错误"观测补偿
        for (int j = 3; j < 5; j++) {
            ekf->P_data[i * EKF_STATE_DIM + j] = 0.0f;
            ekf->P_data[j * EKF_STATE_DIM + i] = 0.0f;
        }
    }
    
    // 3. 将角度严格限制在 -PI 到 PI，防止长时间运行导致三角函数失真
    #define PI_F 3.1415926535f
    while (ekf->X_data[2] > PI_F)  ekf->X_data[2] -= 2.0f * PI_F;
    while (ekf->X_data[2] < -PI_F) ekf->X_data[2] += 2.0f * PI_F;
}