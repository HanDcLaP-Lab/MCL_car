#include "encoder.h"
#include "zf_common_headfile.h"
Encoder encoder_data={0};
KalmanFilter1 K_lf ,K_rf ,K_lb ,K_rb;  

void Encoder_Init(){
    encoder_quad_init(ENCODER_QUAD_lf, ENCODER_QUAD_lf_PHASE_A, ENCODER_QUAD_lf_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD_rf, ENCODER_QUAD_rf_PHASE_A, ENCODER_QUAD_rf_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD_lb, ENCODER_QUAD_lb_PHASE_A, ENCODER_QUAD_lb_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD_rb, ENCODER_QUAD_rb_PHASE_A, ENCODER_QUAD_rb_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    Kalman_Init(&K_lf, 0.05f, 2.0f, 0.0f); // 调低过程噪声Q并增大测量噪声R，增强对极短周期单脉冲跳动的低通平滑效果
    Kalman_Init(&K_rf, 0.05f, 2.0f, 0.0f);
    Kalman_Init(&K_lb, 0.05f, 2.0f, 0.0f);
    Kalman_Init(&K_rb, 0.05f, 2.0f, 0.0f);
}

void Encoder_GetCount(){
    encoder_data.lf = encoder_get_count(ENCODER_QUAD_lf) / CONTROL_DT * RPM_TO_MPS;
    encoder_data.lf = -Kalman_Update(&K_lf, encoder_data.lf) ;
    encoder_clear_count(ENCODER_QUAD_lf);

    encoder_data.rf = encoder_get_count(ENCODER_QUAD_rf) / CONTROL_DT * RPM_TO_MPS;
    encoder_data.rf = Kalman_Update(&K_rf, encoder_data.rf);
    encoder_clear_count(ENCODER_QUAD_rf);

    encoder_data.lb = encoder_get_count(ENCODER_QUAD_lb) / CONTROL_DT * RPM_TO_MPS;
    encoder_data.lb = -Kalman_Update(&K_lb, encoder_data.lb);
    encoder_clear_count(ENCODER_QUAD_lb);

    encoder_data.rb = encoder_get_count(ENCODER_QUAD_rb) / CONTROL_DT * RPM_TO_MPS;
    encoder_data.rb = Kalman_Update(&K_rb, encoder_data.rb);
    encoder_clear_count(ENCODER_QUAD_rb);
}