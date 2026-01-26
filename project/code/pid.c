#include "zf_common_headfile.h"

// 初始化函数
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_i, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_i = max_i;
    pid->out_max = out_max;
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// 重置函数
void PID_Reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// 核心计算函数
float PID_Calculate(PID_t *pid, float error, float dt) {
    // 1. P项计算
    float p_out = pid->kp * error;

    // 2. I项计算
    pid->integral += error * dt;

    // 积分限幅 (Anti-windup)
    if (pid->integral > pid->max_i) {
        pid->integral = pid->max_i;
    } else if (pid->integral < -pid->max_i) {
        pid->integral = -pid->max_i;
    }
    
    float i_out = pid->ki * pid->integral;

    // 3. D项计算
    // 简单的误差微分: (当前误差 - 上次误差) / dt
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error; // 更新历史误差
    
    float d_out = pid->kd * derivative;

    // 4. 总输出计算
    float output = p_out + i_out + d_out;
    pid->output = output;
    // 5. 输出限幅
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < -pid->out_max) {
        output = -pid->out_max;
    }

    return output;
}