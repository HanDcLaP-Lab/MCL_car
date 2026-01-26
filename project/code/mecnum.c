#include "mecnum.h"

#include "zf_common_headfile.h"

float KP=3500.0f,KI=50000.0f,KD=0.0f,MAX_I=0.2f;
// ================== 全局变量 ==================
PID_t pid_lf, pid_rf, pid_lb, pid_rb;
Target_t target_vel = {0};

// ================== 内部辅助函数 ==================

/**
 * @brief 设置单个电机输出
 * @param pwm_ch: PWM通道
 * @param dir_pin: 方向引脚
 * @param output: PID计算出的输出值 (正负代表方向)
 */
void Motor_Set_Output(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, float output) {
    int32_t duty = (int32_t)output;
    if (pwm_ch == MOTOR_RF_PWM || pwm_ch == MOTOR_RB_PWM ) {
        if (duty >= 0) {
            gpio_set_level(dir_pin, 1);
        } else {
            gpio_set_level(dir_pin, 0);
            duty = -duty;
        }
    }
    if(pwm_ch == MOTOR_LB_PWM || pwm_ch == MOTOR_LF_PWM){
        if (duty >= 0) {
            gpio_set_level(dir_pin, 0);
        } else {
            gpio_set_level(dir_pin, 1);
            duty = -duty;
        }
    }

    // 限幅
    if (duty > PWM_MAX_M) duty = PWM_MAX_M;

    pwm_set_duty(pwm_ch, (uint32)duty);
}

// ================== 接口函数实现 ==================

void Mecanum_Set_Velocity(float vx, float vy, float wz){
    target_vel.vx = vx;
    target_vel.vy = vy;
    target_vel.wz = wz;
}

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
    // &pid, kp, ki, kd, max_i, out_max
    PID_Init(&pid_lf, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_rf, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_lb, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_rb, KP, KI, KD, MAX_I, OUT_MAX);

    // 4. 初始化目标值
    target_vel.vx = 0;
    target_vel.vy = 0;
    target_vel.wz = 0;
    target_vel.unlock = true;
}

void Mecanum_Stop(void) {
    target_vel.unlock = false;

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

void Mecanum_Unlock(void) {
    target_vel.unlock = true;

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
    Encoder_GetCount();

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
    out_lf = PID_Calculate(&pid_lf, target_v_lf - encoder_data.lf, CONTROL_DT);
    out_rf = PID_Calculate(&pid_rf, target_v_rf - encoder_data.rf, CONTROL_DT);
    out_lb = PID_Calculate(&pid_lb, target_v_lb - encoder_data.lb, CONTROL_DT);
    out_rb = PID_Calculate(&pid_rb, target_v_rb - encoder_data.rb, CONTROL_DT);

    // 4. 执行电机控制
    if (target_vel.unlock == true) {
        Motor_Set_Output(MOTOR_LF_PWM, MOTOR_LF_DIR, out_lf);
        Motor_Set_Output(MOTOR_RF_PWM, MOTOR_RF_DIR, out_rf);
        Motor_Set_Output(MOTOR_LB_PWM, MOTOR_LB_DIR, out_lb);
        Motor_Set_Output(MOTOR_RB_PWM, MOTOR_RB_DIR, out_rb);
    }
}
// 为方便显示，取mm/s
void Current_speed_display(void) {
    printf("%d,%d,%d,%d\n", (int16_t)(1000 * encoder_data.lf), (int16_t)(1000 * encoder_data.rf),
                             (int16_t)(1000 * encoder_data.lb), (int16_t)(1000 * encoder_data.rb));
    // printf("%d\r\n", (int16_t)(1000 * encoder_data.lb));
    // printf("%d\r\n", (int16_t)(1000 * encoder_data.rf));
    // printf("%d\r\n", (int16_t)(1000 * encoder_data.rb));
    // // printf("RF: %d\r\n", (int16_t)(1000 *encoder_data.rf));
    // printf("LB: %.2f\r\n", 100 * (float)(encoder_data.lb));
    // printf("RB: %.2f\r\n", 100 * (float)(encoder_data.rb));
}