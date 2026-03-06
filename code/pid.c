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
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
}

// 重置函数
void PID_Reset(PID_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output = 0.0f;
}

// 核心计算函数
float PID_Calculate(PID_t *pid, float error, float dt) {
    //若接近目标状态，直接归零PID，防止疯转
    if (fabsf(error) < 0.001f) {
        pid->integral = 0;
        pid->prev_error = 0;
        pid->output = 0;
        return 0; // 直接返回 0，不进行后续计算
    }

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

// 增量式 PID 计算函数
float PID_Calculate_Incremental(PID_t *pid, float error, float dt) {
    // 增量式公式: Delta_U = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2e(k-1)+e(k-2))/dt
    
    float p_term = pid->kp * (error - pid->prev_error);
    float i_term = pid->ki * error * dt;
    float d_term = 0.0f;
    
    if (dt > 0.000001f) {
        d_term = pid->kd * (error - 2 * pid->prev_error + pid->prev_prev_error) / dt;
    }

    // 计算增量
    float delta_output = p_term + i_term + d_term;

    // 累加到输出 (Output 充当了积分器的角色)
    pid->output += delta_output;

    // 输出限幅
    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    } else if (pid->output < -pid->out_max) {
        pid->output = -pid->out_max;
    }

    // 更新历史误差
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;

    return pid->output;
}