#ifndef _MECANUM_H
#define _MECANUM_H

#include "zf_common_headfile.h"
#include "pid.h"

// ================== 车辆物理参数定义 ==================
// 请根据实际小车尺寸修改 (米)
#define CAR_L           0.10f   // 前后轮轴距的一半 (Half Wheel Base)
#define CAR_W           0.09f   // 左右轮距的一半 (Half Track Width)
#define WHEEL_RADIUS    0.028f   // 轮子半径
#define PWM_MAX_M       7000.0f  // PWM 最大占空比。理论上限10000，来自PWM_DUTY_MAX

// 控制周期 (秒)
#define CONTROL_DT      0.001f   

#define KP 280.0f
#define KI 150.0f
#define KD 0.0f
#define MAX_I 30.0f
#define OUT_MAX 60.0f
// ================== 硬件引脚定义 ==================
// 电机 PWM 通道定义
#define MOTOR_LF_PWM    TCPWM_CH14_P00_2    // 左前 PWM
#define MOTOR_RF_PWM    TCPWM_CH51_P18_6    // 右前 PWM
#define MOTOR_LB_PWM    TCPWM_CH54_P18_3    // 左后 PWM
#define MOTOR_RB_PWM    TCPWM_CH00_P06_1    // 右后 PWM

// 电机方向引脚定义
#define MOTOR_LF_DIR    P00_3               // 左前 DIR
#define MOTOR_RF_DIR    P18_7               // 右前 DIR
#define MOTOR_LB_DIR    P18_4               // 左后 DIR
#define MOTOR_RB_DIR    P06_3               // 右后 DIR

// ================== 结构体定义 ==================
typedef struct {
    float vx;       // X轴速度 (m/s)
    float vy;       // Y轴速度 (m/s)
    float wz;       // 自转角速度 (rad/s)

    bool unlock;   //1解锁0上锁
} Target_t;

// ================== 函数声明 ==================

/**
 * @brief 初始化麦轮底盘 (GPIO, PWM, PID)
 */
void Mecanum_Init(void);

/**
 * @brief 设置底盘目标速度
 * @param vx: 前进速度 (m/s)
 * @param vy: 横移速度 (m/s), 左正右负
 * @param wz: 旋转速度 (rad/s), 逆时针正
 */
void Mecanum_Set_Velocity(float vx, float vy, float wz);

/**
 * @brief 底盘控制循环，建议在定时器中断中调用 (周期需与 CONTROL_DT 一致)
 */
void Mecanum_Control_Loop(void);

/**
 * @brief 停止所有电机
 */
void Mecanum_Stop(void);

void Current_speed_display(void);

#endif // _MECANUM_H