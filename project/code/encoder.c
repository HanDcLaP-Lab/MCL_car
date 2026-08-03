#include "encoder.h"
#include "zf_common_headfile.h"
Encoder encoder_data={0};
KalmanFilter1 K_lf ,K_rf ,K_lb ,K_rb;

void Encoder_Init(){
    encoder_quad_init(ENCODER_QUAD_lf, ENCODER_QUAD_lf_PHASE_A, ENCODER_QUAD_lf_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD_rf, ENCODER_QUAD_rf_PHASE_A, ENCODER_QUAD_rf_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD_lb, ENCODER_QUAD_lb_PHASE_A, ENCODER_QUAD_lb_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    encoder_quad_init(ENCODER_QUAD_rb, ENCODER_QUAD_rb_PHASE_A, ENCODER_QUAD_rb_PHASE_B);  // 初始化编码器模块与引脚 正交编码器模式
    // [修改] X4+5ms窗口后量化方差=(0.0373)^2/12≈1.2e-4 (m/s)^2：R=3e-4、Q=5e-5 为初始值，
    // 静置波动>±0.02 m/s 则 R 加倍；阶跃收敛>0.5s 则 Q 增大/R 减小 (见调参指南)
    Kalman_Init(&K_lf, 5e-5f, 3e-4f, 0.0f);
    Kalman_Init(&K_rf, 5e-5f, 3e-4f, 0.0f);
    Kalman_Init(&K_lb, 5e-5f, 3e-4f, 0.0f);
    Kalman_Init(&K_rb, 5e-5f, 3e-4f, 0.0f);
}

// 5ms 滑动窗口：环形缓冲保存最近 5 个 1ms 计数增量，求和后换算速度
// 窗口分辨率 = 1 计数/5ms = 0.0373 m/s，显著优于单 tick 的量化噪声
static int16_t win_hist_lf[ENCODER_WINDOW_TICKS] = {0};
static int16_t win_hist_rf[ENCODER_WINDOW_TICKS] = {0};
static int16_t win_hist_lb[ENCODER_WINDOW_TICKS] = {0};
static int16_t win_hist_rb[ENCODER_WINDOW_TICKS] = {0};
static int32_t win_sum_lf = 0, win_sum_rf = 0, win_sum_lb = 0, win_sum_rb = 0;
static uint8_t win_idx = 0;

void Encoder_GetCount(){
    // 1. 读增量并更新滑动窗口 (每 1ms 调用一次，增量可正可负)
    int16_t cnt_lf = encoder_get_count(ENCODER_QUAD_lf);
    win_sum_lf += (int32_t)cnt_lf - win_hist_lf[win_idx];
    win_hist_lf[win_idx] = cnt_lf;

    int16_t cnt_rf = encoder_get_count(ENCODER_QUAD_rf);
    win_sum_rf += (int32_t)cnt_rf - win_hist_rf[win_idx];
    win_hist_rf[win_idx] = cnt_rf;

    int16_t cnt_lb = encoder_get_count(ENCODER_QUAD_lb);
    win_sum_lb += (int32_t)cnt_lb - win_hist_lb[win_idx];
    win_hist_lb[win_idx] = cnt_lb;

    int16_t cnt_rb = encoder_get_count(ENCODER_QUAD_rb);
    win_sum_rb += (int32_t)cnt_rb - win_hist_rb[win_idx];
    win_hist_rb[win_idx] = cnt_rb;

    // 2. 窗口计数换算速度 (5ms) 后过卡尔曼
    encoder_data.lf = (float)win_sum_lf / (ENCODER_WINDOW_TICKS * CONTROL_DT) * RPM_TO_MPS;
    encoder_data.lf = Kalman_Update(&K_lf, encoder_data.lf);

    encoder_data.rf = (float)win_sum_rf / (ENCODER_WINDOW_TICKS * CONTROL_DT) * RPM_TO_MPS;
    encoder_data.rf = -Kalman_Update(&K_rf, encoder_data.rf);

    encoder_data.lb = (float)win_sum_lb / (ENCODER_WINDOW_TICKS * CONTROL_DT) * RPM_TO_MPS;
    encoder_data.lb = Kalman_Update(&K_lb, encoder_data.lb);

    encoder_data.rb = (float)win_sum_rb / (ENCODER_WINDOW_TICKS * CONTROL_DT) * RPM_TO_MPS;
    encoder_data.rb = -Kalman_Update(&K_rb, encoder_data.rb);

    // 3. 游标推进 (四轮共用同一 tick 相位)
    win_idx = (win_idx + 1) % ENCODER_WINDOW_TICKS;
}

void Encoder_Test_Print(void) {
    // 为了防止打印过快卡死主循环或占用过多总线资源，加一个简单的分频计数器
    static uint16_t print_cnt = 0;
    if (++print_cnt >= 50) { 
        print_cnt = 0;
        // 打印四个轮子的过滤后速度 (单位: m/s)
        printf("%.2f ,%.2f ,%.2f ,%.2f\n", 
               encoder_data.lf, encoder_data.rf, encoder_data.lb, encoder_data.rb);
    }
}