#ifndef _MECANUM_H
#define _MECANUM_H

#include "zf_common_headfile.h"
#include "pid.h"

// ================== 车辆物理参数定义 ==================
// 请根据实际小车尺寸修改 (米)
#define CAR_L           0.10f   // 前后轮轴距的一半 (Half Wheel Base)
#define CAR_W           0.09f   // 左右轮距的一半 (Half Track Width)
#define WHEEL_RADIUS    0.028f   // 轮子半径
#define PWM_MAX_M       5000.0f  // 四轮 PWM 最大占空比。理论上限10000，来自PWM_DUTY_MAX
#define TARGET_SPEED    0.75f     // 目标速度 (m/s)

// 控制周期 (秒)
#define CONTROL_DT      0.001f
#define VISUAL_DT       0.020f

// 【新增】加速度限制 (单位: m/s^2 和 rad/s^2)
#define MAX_ACCEL_X  15.0f
#define MAX_ACCEL_Y  15.0f
#define MAX_ACCEL_W  10.0f


#define OUT_MAX PWM_MAX_M       // 四轮速度环输出上限与实际 PWM 限幅保持一致
// ================== 硬件引脚定义 ==================
// 电机 PWM 通道定义
#define MOTOR_LF_PWM    TCPWM_CH50_P18_7    // 左前 PWM
#define MOTOR_RF_PWM    TCPWM_CH13_P00_3    // 右前 PWM
#define MOTOR_LB_PWM    TCPWM_CH54_P18_3    // 左后 PWM
#define MOTOR_RB_PWM    TCPWM_CH52_P18_5    // 右后 PWM

// 电机方向引脚定义
#define MOTOR_LF_DIR    P18_6               // 左前 DIR
#define MOTOR_RF_DIR    P00_2               // 右前 DIR
#define MOTOR_LB_DIR    P18_4               // 左后 DIR
#define MOTOR_RB_DIR    P06_3               // 右后 DIR

// 记忆/滑行参数
#define COAST_HOLD_MS  600U      // 目标丢失后软滑行保持时间 (ms)
#define TRACK_FRAMES_MAX        50U   // 可靠跟踪置信度累计上限 (帧数, 50帧≈1s@50Hz)
#define TRACK_FRAMES_LOCK_THRESHOLD 8U // 判定已有效锁定所需的可靠跟踪帧数 (8帧≈160ms@50Hz)

// 信标跳变检测
#define JUMP_THRESHOLD_MIN      50.0f   // 跳变检测阈值下限 (cm)
#define JUMP_THRESHOLD_MAX      200.0f  // 跳变检测阈值上限 (cm)
#define JUMP_SCALE_COEF         0.4f    // 跳变阈值缩放系数 (× car_dist)
#define MERGE_COAST_MS          600U    // 信标跳变滑行持续时间 (ms)

// 边缘判定
#define EDGE_DIST_CM            200.0f  // 信标距画面中心距离分界 (cm)，>此值视为边缘
#define DASH_MS_MIN             200U    // 加上固定补偿后的盲冲时长下限 (ms)
#define DASH_MS_MAX             700U    // 加上固定补偿后的盲冲时长上限 (ms)
#define STATE4_DASH_EXTRA_MS    100     // state4 融合盲冲在计算时长基础上固定增加 100ms
#define DASH_HISTORY_SIZE       8U      // state4 盲冲使用最近多帧 state3 位置做等速拟合
#define DASH_HISTORY_MAX_AGE_MS 200U    // 只使用足够新的历史帧，避免旧目标污染融合盲冲
#define DASH_HISTORY_MIN_SAMPLES 4U     // 等速拟合至少需要的有效样本数
#define DASH_HISTORY_SAMPLE_JUMP_BASE_CM 12.0f // 相邻帧允许的基础视觉抖动 (cm)
#define DASH_HISTORY_MAX_REL_SPEED_CM_S 200.0f // 相邻帧相对位置变化速度上限 (cm/s)
#define DASH_HISTORY_MAX_DIR_CHANGE_COS 0.94f // 拟合方向相对上一速度方向最多改变约20度
#define DASH_HISTORY_MIN_CLOSING_SPEED_MPS 0.20f // 可信接近速度下限 (m/s)
#define DASH_HISTORY_MAX_CLOSING_SPEED_MPS 1.20f // 可信接近速度上限 (m/s)

#include "chassis_arm.h"

// ================== 主循环存活看门狗 ==================
// 主循环每轮刷新 main_loop_heartbeat_ms = sys_time_ms。若 1ms ISR 发现距上次刷新
// 超过 MAINLOOP_STALL_MS，则判定主循环卡死并在 ISR 内强制切断动力(等效未武装)。
// 这填补了"主循环挂死时 car_en / 通信看门狗都无法执行"的致命安全空窗。
#define MAINLOOP_STALL_MS   10U

extern float f_t;
extern float smooth_vx, smooth_vy, smooth_wz;
extern volatile uint32_t sys_time_ms;
extern volatile uint32_t main_loop_heartbeat_ms;
// ================== 结构体定义 ==================
typedef struct {
    float vx;       // X轴速度 (m/s)
    float vy;       // Y轴速度 (m/s)
    float wz;       // 自转角速度 (rad/s)

    float v_lf,v_rf,v_lb,v_rb;
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
extern float ang_out;
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

void Current_speed_display(void);

#endif // _MECANUM_H
