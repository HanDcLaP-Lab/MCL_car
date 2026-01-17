#include "mecanum.h"
#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"

// 引入编码器头文件 (如果使用其他编码器请修改)
#include "zf_device_menc15a.h" 

// ================== 全局变量 ==================
PID_t pid_lf, pid_rf, pid_lb, pid_rb;
Chassis_Target_t target_vel = {0};


float measure_speed_lf = 0.0f;
float measure_speed_rf = 0.0f;
float measure_speed_lb = 0.0f;
float measure_speed_rb = 0.0f;

// ================== 内部辅助函数 ==================

/**
 * @brief 设置单个电机输出
 * @param pwm_ch: PWM通道
 * @param dir_pin: 方向引脚
 * @param output: PID计算出的输出值 (正负代表方向)
 */
static void Motor_Set_Output(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, float output) {
    int32_t duty = (int32_t)output;
    
    if (duty >= 0) {
        gpio_set_level(dir_pin, 0); // 假设 0 为正转，需根据实际接线调整
    } else {
        gpio_set_level(dir_pin, 1);
        duty = -duty;
    }

    // 限幅
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    
    pwm_set_duty(pwm_ch, (uint32)duty);
}

/**
 * @brief 获取当前车轮速度 (m/s)
 * @note  此处需要根据实际编码器驱动进行填充
 */
static void Update_Encoder_Feedback(void) {
    // 示例：读取 MENC15A 编码器数据 (仅作演示，因为 menc15a 库只定义了2个模块)
    // 实际使用时，请替换为读取4个编码器的代码
    
    // float rpm_to_mps = (2 * 3.14159f * WHEEL_RADIUS) / 60.0f;
    
    // 假设你有4个编码器读取函数，在此处更新 measure_speed_xx
    // measure_speed_lf = Encoder_Get_Speed_LF() * rpm_to_mps;
    // measure_speed_rf = Encoder_Get_Speed_RF() * rpm_to_mps;
    // measure_speed_lb = Encoder_Get_Speed_LB() * rpm_to_mps;
    // measure_speed_rb = Encoder_Get_Speed_RB() * rpm_to_mps;
}

// ================== 接口函数实现 ==================

void Mecanum_Init(void) {
    // 1. 初始化电机 GPIO (方向引脚)
    gpio_init(MOTOR_LF_DIR, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_RF_DIR, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_LB_DIR, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_RB_DIR, GPO, 0, GPO_PUSH_PULL);

    // 2. 初始化电机 PWM (频率 17kHz)
    pwm_init(MOTOR_LF_PWM, 17000, 0);
    pwm_init(MOTOR_RF_PWM, 17000, 0);
    pwm_init(MOTOR_LB_PWM, 17000, 0);
    pwm_init(MOTOR_RB_PWM, 17000, 0);

    // 3. 初始化 PID
    // 参数: pid, kp, ki, kd, max_i, out_max
    PID_Init(&pid_lf, DEF_KP, DEF_KI, DEF_KD, DEF_MAX_I, DEF_OUT_MAX);
    PID_Init(&pid_rf, DEF_KP, DEF_KI, DEF_KD, DEF_MAX_I, DEF_OUT_MAX);
    PID_Init(&pid_lb, DEF_KP, DEF_KI, DEF_KD, DEF_MAX_I, DEF_OUT_MAX);
    PID_Init(&pid_rb, DEF_KP, DEF_KI, DEF_KD, DEF_MAX_I, DEF_OUT_MAX);
    
    // 4. 初始化目标值
    target_vel.vx = 0;
    target_vel.vy = 0;
    target_vel.wz = 0;
}

void Mecanum_Set_Velocity(float vx, float vy, float wz) {
    target_vel.vx = vx;
    target_vel.vy = vy;
    target_vel.wz = wz;
}

void Mecanum_Stop(void) {
    Mecanum_Set_Velocity(0, 0, 0);
    pwm_set_duty(MOTOR_LF_PWM, 0);
    pwm_set_duty(MOTOR_RF_PWM, 0);
    pwm_set_duty(MOTOR_LB_PWM, 0);
    pwm_set_duty(MOTOR_RB_PWM, 0);
    
    // 重置 PID 积分项
    PID_Reset(&pid_lf);
    PID_Reset(&pid_rf);
    PID_Reset(&pid_lb);
    PID_Reset(&pid_rb);
}

void Mecanum_Control_Loop(void) {
    float target_v_lf, target_v_rf, target_v_lb, target_v_rb;
    float out_lf, out_rf, out_lb, out_rb;

    // 1. 获取反馈速度
    Update_Encoder_Feedback();

    // 2. 运动学逆解算 (Inverse Kinematics)
    // 根据车身坐标系 V_x, V_y, Omega 计算四个轮子的线速度
    // 注意：这里的正负号取决于电机安装方向和轮子类型 (A/B轮布局)
    // 典型布局：左前/右后为A轮，右前/左后为B轮
    float center_v = target_vel.wz * (CAR_L + CAR_W);
    
    target_v_lf = target_vel.vx - target_vel.vy - center_v;
    target_v_rf = target_vel.vx + target_vel.vy + center_v;
    target_v_lb = target_vel.vx + target_vel.vy - center_v;
    target_v_rb = target_vel.vx - target_vel.vy + center_v;

    // 3. PID 计算
    out_lf = PID_Calculate(&pid_lf, target_v_lf - measure_speed_lf, CONTROL_DT);
    out_rf = PID_Calculate(&pid_rf, target_v_rf - measure_speed_rf, CONTROL_DT);
    out_lb = PID_Calculate(&pid_lb, target_v_lb - measure_speed_lb, CONTROL_DT);
    out_rb = PID_Calculate(&pid_rb, target_v_rb - measure_speed_rb, CONTROL_DT);

    // 4. 执行电机控制
    Motor_Set_Output(MOTOR_LF_PWM, MOTOR_LF_DIR, out_lf);
    Motor_Set_Output(MOTOR_RF_PWM, MOTOR_RF_DIR, out_rf);
    Motor_Set_Output(MOTOR_LB_PWM, MOTOR_LB_DIR, out_lb);
    Motor_Set_Output(MOTOR_RB_PWM, MOTOR_RB_DIR, out_rb);
}