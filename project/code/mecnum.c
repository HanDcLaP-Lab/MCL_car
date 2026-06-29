#include "mecnum.h"

#include "zf_common_headfile.h"
#include <math.h>
float f_t = 0;
// [参数调整] 针对增量式PID (dt=0.001s) 的调优参数
// KP=3500: 0.5m/s 误差时提供 1750 的基础PWM，确保启动有力
// KI=3000: 0.5m/s 误差时每秒增加 1500 PWM (3000*0.5*0.001*1000)，消除静差只需约0.5-1秒
// KD=0: 速度环通常不需要微分项，除非超调严重
float KP=3500.0f, KI=20000.0f, KD=0.0f, MAX_I=4500.0f;

// ================== 全局变量 ==================
PID_t pid_lf, pid_rf, pid_lb, pid_rb;//速度环pid
PID_t pid_yaw_hold;//角度环pid
PID_t pid_yaw_rate;
// [位置环参数] (暂未启用，位置环控制待实现)
// float POS_KP=0.015f, POS_KI=0.0f, POS_KD=0.0f, POS_MAX_I=0.5f, POS_OUT_MAX=1.0f;

// [参数调整] 
float YAW_KP=0.16f, YAW_KI=0.0f, YAW_KD=0.01f, YAW_MAX_I=10.0f, YAW_OUT_MAX=1.6f;

float YAW_RATE_KP=1.5f, YAW_RATE_KI=0.0f, YAW_RATE_KD=0.0f, YAW_RATE_MAX_I=1.0f, YAW_RATE_OUT_MAX=1.5f;
Target_t target_vel = {0};//目标运行情况
Motor_Output_t motor_output = {0};

// 【新增】斜坡函数相关的平滑速度变量
float smooth_vx = 0.0f;
float smooth_vy = 0.0f;
float smooth_wz = 0.0f;

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

/**
 * @brief PID输出(PWM)等比例缩放防饱和
 * @note  当某一个电机的计算PWM超过硬件最大量程时，将四个电机的PWM等比例缩小，保持推力矢量方向不变。
 * @param out_lf 左前轮PWM计算值指针
 * @param out_rf 右前轮PWM计算值指针
 * @param out_lb 左后轮PWM计算值指针
 * @param out_rb 右后轮PWM计算值指针
 * @param max_pwm 硬件允许的最大PWM绝对值 (例如 PWM_MAX_M)
 */
void PWM_Equal_Proportion_Scale(float *out_lf, float *out_rf, float *out_lb, float *out_rb, float max_pwm) {
    float max_val = 0.0f;
    float abs_lf = fabsf(*out_lf);
    float abs_rf = fabsf(*out_rf);
    float abs_lb = fabsf(*out_lb);
    float abs_rb = fabsf(*out_rb);

    // 1. 找出四个轮子PWM输出中的最大绝对值
    max_val = abs_lf > abs_rf ? abs_lf : abs_rf;
    max_val = max_val > abs_lb ? max_val : abs_lb;
    max_val = max_val > abs_rb ? max_val : abs_rb;

    // 2. 如果最大值超过了硬件允许的量程，则四个轮子等比例缩小
    if (max_val > max_pwm) {
        float scale = max_pwm / max_val;
        *out_lf *= scale;
        *out_rf *= scale;
        *out_lb *= scale;
        *out_rb *= scale;
    }
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
    // [临时-引脚冲突] MOTOR_RB_PWM(P06_1) 与 UART1_TX_P06_1 冲突, 双向通讯期间暂停右后电机 PWM
    // pwm_init(MOTOR_RB_PWM, 17000, 0);

    // 3. 初始化 PID
    // &pid, kp, ki, kd, max_i, out_max
    PID_Init(&pid_lf, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_rf, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_lb, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_rb, KP, KI, KD, MAX_I, OUT_MAX);
    PID_Init(&pid_yaw_hold, YAW_KP, YAW_KI, YAW_KD, YAW_MAX_I, YAW_OUT_MAX);
    PID_Init(&pid_yaw_rate, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, YAW_RATE_MAX_I, YAW_RATE_OUT_MAX);

    // 4. 初始化目标值
    target_vel.vx = 0;
    target_vel.vy = 0;
    target_vel.wz = 0;
}

volatile uint32_t sys_time_ms = 0;

void Mecanum_Control_Loop(void) {
    sys_time_ms++;

    // 1. 获取反馈速度
    Encoder_GetCount();

    // 检查 IMU 是否校准完毕：未校准则锁定底盘并退出。
    // Block/Unblock 幂等，校准期间每 tick 调用也不会重复执行停车清理。
    if(imu_car_rc_data.is_calibrated == 1){
        Chassis_Unblock(DISARM_UNCALIBRATED);
    }else{
        Chassis_Block(DISARM_UNCALIBRATED);
        return;
    }

    // ISR 级时间刹车检查 (dash/coast/merge 到期处理)，由 car_image.c 实现
    Visual_Brake_Check();
    // ==========================================================
    // 【核心一】只对“用户目标指令”进行斜坡平滑 (防起步打滑)
    // 根据设定的最大加速度，限制每 1ms (CONTROL_DT) 的速度变化量
    // ==========================================================
    if (Chassis_Is_Armed()) {
        float step_x = MAX_ACCEL_X * CONTROL_DT;
        float step_y = MAX_ACCEL_Y * CONTROL_DT;
        float step_w = MAX_ACCEL_W * CONTROL_DT;

        // X轴 (前后) 速度平滑
        if (target_vel.vx > smooth_vx + step_x) smooth_vx += step_x;
        else if (target_vel.vx < smooth_vx - step_x) smooth_vx -= step_x;
        else smooth_vx = target_vel.vx;

        // Y轴 (左右) 速度平滑
        if (target_vel.vy > smooth_vy + step_y) smooth_vy += step_y;
        else if (target_vel.vy < smooth_vy - step_y) smooth_vy -= step_y;
        else smooth_vy = target_vel.vy;

        // Z轴 (自转) 速度平滑：注意，这里只平滑外部下发的目标 target_vel.wz
        if (target_vel.wz > smooth_wz + step_w) smooth_wz += step_w;
        else if (target_vel.wz < smooth_wz - step_w) smooth_wz -= step_w;
        else smooth_wz = target_vel.wz;
    } else {
        // 如果未解锁，平滑速度强制归零
        smooth_vx = 0.0f;
        smooth_vy = 0.0f;
        smooth_wz = 0.0f;
    }
    
    // ==========================================================
    // 【核心二】角度-角速度 串级双环 (死死咬住航向，绝不偏转)
    // ==========================================================
    float final_wz = smooth_wz; // 默认采用平滑后的目标自转速度
    
    if (Chassis_Is_Armed()) {
            // 将陀螺仪实际角速度从 deg/s 转换为 rad/s，统一量纲！
            float current_rate_rad = imu_car_rc_data.yaw_rate * ((float)M_PI / 180.0f);

        // 如果外部没有要求自转 (判断平滑后的 smooth_wz 近似为0)，启动 Yaw 锁死
        if (fabsf(smooth_wz) < 0.05f) {
            
            // --- 外环：角度控制 (只管方向) ---
            float yaw_error = 0.0f - imu_car_rc_data.yaw_total; 
            
            // 角度死区：1.5度以内放弃纠偏，防止原地鬼畜发热
            if (fabsf(yaw_error) < 1.5f) {
                yaw_error = 0.0f;
            }
            // 外环输出 = 期望车体转多快 (rad/s)
            float target_yaw_rate = PID_Calculate(&pid_yaw_hold, yaw_error, CONTROL_DT);

            // --- 内环：角速度控制 (抵抗打滑) ---
            float rate_error = target_yaw_rate - current_rate_rad;
            // 内环输出 = 给逆解算的最终瞬间补偿量 (final_wz 绝对不能再过斜坡平滑)
            final_wz = PID_Calculate(&pid_yaw_rate, rate_error, CONTROL_DT);

        } else {
            // 如果外部发送了主动旋转命令，外环暂停，内环直接跟踪平滑后的角速度
            PID_Reset(&pid_yaw_hold); 
            float rate_error = smooth_wz - current_rate_rad;
            final_wz = PID_Calculate(&pid_yaw_rate, rate_error, CONTROL_DT);
        }
    }
    f_t = final_wz; // 记录用于调试输出

    // ==========================================================
    // 【核心三】运动学逆解算 (包含重心前移与后轮抓地力补偿)
    // ==========================================================
    float offset_x = 0.02f; // 重心前移量 (2cm)，需根据实车微调
    
    // 计算以新重心为原点，前后轮的实际纵向力臂
    float L_front = CAR_L - offset_x;
    float L_rear  = CAR_L + offset_x;
    
    // 使用闭环输出的 final_wz 直接计算旋转所需的差速
    float center_v_front = final_wz * (L_front + CAR_W);
    float center_v_rear  = final_wz * (L_rear  + CAR_W);
    
    // 侧向移动时，给容易打滑的后轮增加推力权重 (10%~15%)
    float vy_front = smooth_vy;
    float vy_rear  = smooth_vy * 1.10f; 

    // 逆解算公式应用非对称参数
    target_vel.v_lf = smooth_vx - vy_front + center_v_front;
    target_vel.v_rf = smooth_vx + vy_front - center_v_front;
    
    target_vel.v_lb = smooth_vx + vy_rear  + center_v_rear;
    target_vel.v_rb = smooth_vx - vy_rear  - center_v_rear;

    // ==========================================================
    // 【核心四】底层轮速 PID 计算与防饱和机制
    // ==========================================================
    float err_lf = target_vel.v_lf - encoder_data.lf;
    float err_rf = target_vel.v_rf - encoder_data.rf;
    float err_lb = target_vel.v_lb - encoder_data.lb;
    float err_rb = target_vel.v_rb - encoder_data.rb;

    if (Chassis_Is_Armed()) {
        // 计算原始增量 PID 输出 (此时绝不能限幅)
        motor_output.lf = PID_Calculate_Incremental(&pid_lf, err_lf, CONTROL_DT);
        motor_output.rf = PID_Calculate_Incremental(&pid_rf, err_rf, CONTROL_DT);
        motor_output.lb = PID_Calculate_Incremental(&pid_lb, err_lb, CONTROL_DT);
        motor_output.rb = PID_Calculate_Incremental(&pid_rb, err_rb, CONTROL_DT);

        // 简单的误差死区处理，防止静止时电机高频异响抖动
        if (fabsf(target_vel.v_lf) < 0.01f && fabsf(err_lf) < 0.03f) motor_output.lf = 0;
        if (fabsf(target_vel.v_rf) < 0.01f && fabsf(err_rf) < 0.03f) motor_output.rf = 0;
        if (fabsf(target_vel.v_lb) < 0.01f && fabsf(err_lb) < 0.03f) motor_output.lb = 0;
        if (fabsf(target_vel.v_rb) < 0.01f && fabsf(err_rb) < 0.03f) motor_output.rb = 0;

        // 步骤 1：PWM 等比例缩放，保证打滑或极限加速时，推力矢量不发生畸变
        PWM_Equal_Proportion_Scale(&motor_output.lf, &motor_output.rf, 
                                   &motor_output.lb, &motor_output.rb, 
                                   (float)PWM_MAX_M);

        // 步骤 2：反向写回内部状态 (Anti-Windup)，保证无论怎么限幅，PID 刹车永远迅猛无延迟
        pid_lf.output = motor_output.lf;
        pid_rf.output = motor_output.rf;
        pid_lb.output = motor_output.lb;
        pid_rb.output = motor_output.rb;
        
    } else {
        motor_output.lf = 0;
        motor_output.rf = 0;
        motor_output.lb = 0;
        motor_output.rb = 0;
    }

    // ==========================================================
    // 5. 最终执行电机控制
    // ==========================================================
    if (Chassis_Is_Armed()) {
        Motor_Set_Output(MOTOR_LF_PWM, MOTOR_LF_DIR, motor_output.lf);
        Motor_Set_Output(MOTOR_RF_PWM, MOTOR_RF_DIR, motor_output.rf);
        Motor_Set_Output(MOTOR_LB_PWM, MOTOR_LB_DIR, motor_output.lb);
        // [临时-引脚冲突] 右后电机 PWM(P06_1) 与 UART1 TX 冲突, 双向通讯期间暂停输出
        // Motor_Set_Output(MOTOR_RB_PWM, MOTOR_RB_DIR, motor_output.rb);
    }else{
        pwm_set_duty(MOTOR_LF_PWM, 0);
        pwm_set_duty(MOTOR_RF_PWM, 0);
        pwm_set_duty(MOTOR_LB_PWM, 0);
        // [临时-引脚冲突] 同上, 暂停右后电机 PWM
        // pwm_set_duty(MOTOR_RB_PWM, 0);
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
