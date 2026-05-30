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
#define TARGET_SPEED    0.6f     // 目标速度 (m/s)

// 控制周期 (秒)
#define CONTROL_DT      0.001f   
#define VISUAL_DT       0.020f

// 【新增】加速度限制 (单位: m/s^2 和 rad/s^2)
// 例如: 2.0m/s^2 意味着从 0 加速到 1.0m/s 需要 0.5秒
#define MAX_ACCEL_X  8.0f
#define MAX_ACCEL_Y  8.0f
#define MAX_ACCEL_W  10.0f


#define OUT_MAX 7000.0f
// ================== 硬件引脚定义 ==================
// 电机 PWM 通道定义
#define MOTOR_LF_PWM    TCPWM_CH50_P18_7    // 左前 PWM
#define MOTOR_RF_PWM    TCPWM_CH13_P00_3    // 右前 PWM
#define MOTOR_LB_PWM    TCPWM_CH54_P18_3    // 左后 PWM
#define MOTOR_RB_PWM    TCPWM_CH00_P06_1    // 右后 PWM

// 电机方向引脚定义
#define MOTOR_LF_DIR    P18_6               // 左前 DIR
#define MOTOR_RF_DIR    P00_2               // 右前 DIR
#define MOTOR_LB_DIR    P18_4               // 左后 DIR
#define MOTOR_RB_DIR    P06_3               // 右后 DIR

//记忆功能
// 记忆与视觉边界功能
#define COAST_CNT      20        // 目标丢失后的记忆滑行帧数 (每帧约20ms，3帧即60ms)
#define EDGE_CNT  30
#define COAST_DECAY    1.0f   // 速度衰减系数 (每次循环衰减)
//#define EDGE_X         50.0f    // 前后方向边缘视野界限 (cm)
#define EDGE_Y         300.0f    // 左右方向边缘视野界限 (cm)

// 信标合并滑行参数
#define MERGE_DIST_THRESHOLD    28.0f   // 触发合并滑行的车-信标距离上限 (cm)
#define MERGE_JUMP_THRESHOLD    40.0f   // target 坐标跳变检测阈值 (cm)
#define MERGE_COAST_FRAMES      25      // 合并滑行持续帧数 (50Hz 下 25 帧 = 0.5s)

extern volatile int EN;
extern float f_t;
// ================== 结构体定义 ==================
typedef struct {
    float vx;       // X轴速度 (m/s)
    float vy;       // Y轴速度 (m/s)
    float wz;       // 自转角速度 (rad/s)

    float v_lf,v_rf,v_lb,v_rb;

    bool unlock;   //1解锁0上锁
} Target_t;

typedef struct {
    float lf;
    float rf;
    float lb;
    float rb;
} Motor_Output_t;

// ================== 函数声明 ==================
void Motor_Set_Output(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, float output);
/**
 * @brief 初始化麦轮底盘 (GPIO, PWM, PID)
 */
void Mecanum_Init(void);
extern PID_t pid_lf, pid_rf, pid_lb, pid_rb;
extern PID_t pid_yaw_hold;
// [已移除] extern PID_t pid_pos_x, pid_pos_y; — 位置环未实现，pid_pos_x/y 未定义，引用会导致链接错误
extern float KP,KI,KD,MAX_I;
extern float YAW_KP, YAW_KI, YAW_KD, YAW_MAX_I, YAW_OUT_MAX;
extern Target_t target_vel;

// 在 extern PID_t pid_yaw_hold; 下方添加：
extern PID_t pid_yaw_rate;

// 在 extern float YAW_KP... 下方添加内环参数声明：
extern float YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, YAW_RATE_MAX_I, YAW_RATE_OUT_MAX;

extern Motor_Output_t motor_output;
extern float ang_out,dist_out;
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
 * @brief 视觉控制循环，建议在定时器中断中调用 (周期需与 VISUAL_DT 一致)
 */
void Visual_Control_Loop(void);

/**
 * @brief 停止所有电机
 */
void Mecanum_Stop(void);
void Mecanum_Unlock(void);

void Current_speed_display(void);

#endif // _MECANUM_H