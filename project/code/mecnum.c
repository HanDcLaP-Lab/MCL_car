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
static volatile uint8_t large_turn_accel_state = 0U; // 0:关闭 1:等待新指令 2:过渡中

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
    // [CR-18] 三元组跨主循环/ISR 非原子: 短临界区内一次发布完整目标速度
    uint32_t primask = interrupt_global_disable();
    target_vel.vx = vx;
    target_vel.vy = vy;
    target_vel.wz = wz;
    interrupt_global_enable(primask);
}

void Mecanum_Set_Large_Turn_Accel_Limit(uint8_t enable) {
    large_turn_accel_state = enable ? 1U : 0U;
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
    PID_Init(&pid_yaw_hold, YAW_KP, YAW_KI, YAW_KD, YAW_MAX_I, YAW_OUT_MAX);
    PID_Init(&pid_yaw_rate, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, YAW_RATE_MAX_I, YAW_RATE_OUT_MAX);

    // 4. 初始化目标值
    target_vel.vx = 0;
    target_vel.vy = 0;
    target_vel.wz = 0;
}

volatile uint32_t sys_time_ms = 0;
volatile uint32_t main_loop_heartbeat_ms = 0;   // 主循环存活心跳 (由 main_cm4 主循环刷新)

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

    // 主循环存活看门狗：主循环卡死(如板间 RX 排空死循环、某中断阻塞)时，
    // car_en 处理与通信看门狗(都在主循环里)都无法执行，动力会失控。
    // [CR-23] 锁存式: ISR 检测到超时 → 置入 DISARM_MAINLOOP_STALL (停车清理并归零
    // target_vel/smooth_*)。ISR 绝不自动解除该位；只有主循环恢复后解析到一帧
    // 合法新包才解除 (见 main_cm4.c)，杜绝"停一下又沿旧方向跑"。
    if (!TEST_MODE && ((uint32_t)(sys_time_ms - main_loop_heartbeat_ms) > MAINLOOP_STALL_MS)) {
        Chassis_Block(DISARM_MAINLOOP_STALL);    // 幂等; 仅置位跳变时执行一次停车清理
    }
    uint8_t armed = Chassis_Is_Armed();

    // ISR 级时间刹车检查 (dash 到期处理)，由 car_image.c 实现
    Visual_Brake_Check();
    // ==========================================================
    // 【核心一】只对“用户目标指令”进行斜坡平滑 (防起步打滑)
    // 平移速度按总加速度模长限幅，转向时不改变速度增量方向
    // ==========================================================
    if (armed) {
        float delta_vx = target_vel.vx - smooth_vx;
        float delta_vy = target_vel.vy - smooth_vy;
        float delta_speed = sqrtf(delta_vx * delta_vx + delta_vy * delta_vy);
        float max_delta_speed = MAX_ACCEL_LINEAR * CONTROL_DT;
        uint8_t large_turn_state = large_turn_accel_state;
        float step_w = MAX_ACCEL_W * CONTROL_DT;

        if (large_turn_state != 0U) {
            max_delta_speed *= LARGE_TURN_ACCEL_SCALE;
        }
        if (delta_speed > max_delta_speed) {
            float scale = max_delta_speed / delta_speed;
            delta_vx *= scale;
            delta_vy *= scale;
            if (large_turn_state == 1U && large_turn_accel_state == 1U) {
                large_turn_accel_state = 2U;
            }
        } else if (large_turn_state == 2U && large_turn_accel_state == 2U) {
            large_turn_accel_state = 0U;
        }
        smooth_vx += delta_vx;
        smooth_vy += delta_vy;

        // Z轴 (自转) 速度平滑：注意，这里只平滑外部下发的目标 target_vel.wz
        if (target_vel.wz > smooth_wz + step_w) smooth_wz += step_w;
        else if (target_vel.wz < smooth_wz - step_w) smooth_wz -= step_w;
        else smooth_wz = target_vel.wz;
    } else {
        // 如果未解锁，平滑速度强制归零
        smooth_vx = 0.0f;
        smooth_vy = 0.0f;
        smooth_wz = 0.0f;
        large_turn_accel_state = 0U;
    }
    
    // ==========================================================
    // 【核心二】角度-角速度 串级双环 (死死咬住航向，绝不偏转)
    // ==========================================================
    float final_wz = smooth_wz; // 默认采用平滑后的目标自转速度
    
    if (armed) {
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

    if (armed) {
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
    if (armed) {
        Motor_Set_Output(MOTOR_LF_PWM, MOTOR_LF_DIR, motor_output.lf);
        Motor_Set_Output(MOTOR_RF_PWM, MOTOR_RF_DIR, motor_output.rf);
        Motor_Set_Output(MOTOR_LB_PWM, MOTOR_LB_DIR, motor_output.lb);
        Motor_Set_Output(MOTOR_RB_PWM, MOTOR_RB_DIR, motor_output.rb);
    }else{
        pwm_set_duty(MOTOR_LF_PWM, 0);
        pwm_set_duty(MOTOR_RF_PWM, 0);
        pwm_set_duty(MOTOR_LB_PWM, 0);
        pwm_set_duty(MOTOR_RB_PWM, 0);
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
