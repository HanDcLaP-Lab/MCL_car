#ifndef _MECANUM_H
#define _MECANUM_H

#include "zf_common_headfile.h"
#include "pid.h"

// ================== 车辆物理参数定义 ==================
// 请根据实际小车尺寸修改 (单位: 米)
#define CAR_L           0.10f   // 前后轮轴距的一半 (Half Wheel Base)
#define CAR_W           0.09f   // 左右轮距的一半 (Half Track Width)
#define WHEEL_RADIUS    0.028f   // 轮子半径

// 控制周期 (秒), 例如 10ms = 0.01f
#define CONTROL_DT      0.001f   

// ================== 硬件引脚定义 ==================
// 电机 PWM 通道定义 (参考 zf_driver_pwm.h)
#define MOTOR_LF_PWM    TCPWM_CH00_P06_1    // 左前 PWM
#define MOTOR_RF_PWM    TCPWM_CH01_P06_3    // 右前 PWM
#define MOTOR_LB_PWM    TCPWM_CH02_P06_5    // 左后 PWM
#define MOTOR_RB_PWM    TCPWM_CH06_P02_1    // 右后 PWM

// 电机方向引脚定义 (参考 zf_driver_gpio.h)
#define MOTOR_LF_DIR    P06_2               // 左前 DIR
#define MOTOR_RF_DIR    P06_4               // 右前 DIR
#define MOTOR_LB_DIR    P06_6               // 左后 DIR
#define MOTOR_RB_DIR    P02_2               // 右后 DIR

// ================== 结构体定义 ==================
typedef struct {
    float vx;       // X轴速度 (m/s)
    float vy;       // Y轴速度 (m/s)
    float wz;       // 自转角速度 (rad/s)
} Chassis_Target_t;

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

#endif // _MECANUM_H