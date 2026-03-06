#ifndef _PID_H
#define _PID_H

#include "zf_common_headfile.h"

// =================== PID 结构体定义 ===================
typedef struct {
    // --- 参数 (Parameters) ---
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    
    float max_i;        // 积分限幅 (防止积分饱和/Windup)
    float out_max;      // 总输出限幅
    
    // --- 运行时状态 (Runtime State) ---
    float integral;     // 积分累加值
    float prev_error;   // 上一次误差 (用于计算微分)
    float prev_prev_error; // [新增] 上上次误差 (用于增量式PID)
    float output;       //仅作展示
} PID_t;

// =================== 函数声明 ===================

/**
 * 初始化 PID 参数
 * @param pid: 指向 PID 结构体的指针
 * @param kp, ki, kd: PID 参数
 * @param max_i: 积分项的最大值 (绝对值)
 * @param out_max: 总输出的最大值 (绝对值)
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_i, float out_max);

/**
 * 重置 PID 状态 (清除积分和历史误差)
 * 通常在解锁瞬间或切换模式时调用
 */
void PID_Reset(PID_t *pid);

/**
 * 计算 PID 输出
 * @param pid: 指向 PID 结构体的指针
 * @param error: 当前误差 (目标值 - 测量值)
 * @param dt: 控制周期 (单位: 秒)，例如 1ms 传 0.001f
 * @return: PID 计算出的控制量
 */
float PID_Calculate(PID_t *pid, float error, float dt);

/**
 * 计算 增量式 PID 输出 (推荐用于电机速度环)
 * @param pid: 指向 PID 结构体的指针
 * @param error: 当前误差
 * @param dt: 控制周期
 * @return: PID 计算出的控制量 (PWM)
 */
float PID_Calculate_Incremental(PID_t *pid, float error, float dt);

#endif