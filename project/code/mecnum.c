#include "mecnum.h"

#include "zf_common_headfile.h"
#include <math.h>
int EN = 1;
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
// [位置环参数] 1cm误差对应0.015m/s速度
float POS_KP=0.015f, POS_KI=0.0f, POS_KD=0.0f, POS_MAX_I=0.5f, POS_OUT_MAX=1.0f;

// [参数调整] 
float YAW_KP=0.16f, YAW_KI=0.0f, YAW_KD=0.01f, YAW_MAX_I=10.0f, YAW_OUT_MAX=1.6f;

float YAW_RATE_KP=1.5f, YAW_RATE_KI=0.0f, YAW_RATE_KD=0.0f, YAW_RATE_MAX_I=1.0f, YAW_RATE_OUT_MAX=1.5f;
Target_t target_vel = {0};//目标运行情况
Motor_Output_t motor_output = {0};
float ang_out=0,dist_out=0;

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
    target_vel.unlock = true;
}

void Mecanum_Stop(void) {
    target_vel.unlock = false;
    EN = 0;

    smooth_vx = 0.0f;
    smooth_vy = 0.0f;
    smooth_wz = 0.0f;
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
    PID_Reset(&pid_yaw_hold);
    PID_Reset(&pid_yaw_rate);

    motor_output.lf = 0; motor_output.rf = 0;
    motor_output.lb = 0; motor_output.rb = 0;
}

void Mecanum_Unlock(void) {
    target_vel.unlock = true;
    EN = 1;
    smooth_vx = 0.0f;
    smooth_vy = 0.0f;
    smooth_wz = 0.0f;
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
    PID_Reset(&pid_yaw_hold);
    PID_Reset(&pid_pos_x);
    PID_Reset(&pid_pos_y);
    PID_Reset(&pid_yaw_rate);
}
void Visual_Control_Loop(void) {
    static uint16_t lost_cnt = 0;
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    static uint16_t is_edge = 0;
    static uint16_t edge_lost_cnt = 0;

    static uint16_t target_edge_cnt = 0;
    if (target_vel.unlock) {
        // 解析无人机下发的位掩码状态
        // 0: 全丢, 1: 仅小车, 2: 仅信标, 3: 都有
        uint8_t locked_state = (uint8_t)uart_data[5]; 

        // 设定滑行时间: 3 个 VISUAL_DT (假设 VISUAL_DT 为 0.02s，即 60ms)
        //uint32_t coast_max_time = (uint32_t)(3 * VISUAL_DT * 1000); 

        // ==========================================
        // 状态 3：双目标锁定 (正常追踪)
        // ==========================================
        if (locked_state == 3) {
            float dist = 0.0f, angle = 0.0f;
            Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

            ang_out = angle;
            dist_out = dist;

            float current_speed = TARGET_SPEED;
             if (dist < 20.0f)   current_speed = 0.35f; // 死区刹车
            // } else if (dist < 35.0f) {
            //     current_speed = TARGET_SPEED * (dist / 35.0f); // 比例减速
            //     if (current_speed < 0.15f) current_speed = 0.15f; 
            // }

            float angle_rad = angle * (float)(3.1415926f / 180.0f);
            float target_speed_x = current_speed * cosf(angle_rad);
            float target_speed_y = current_speed * sinf(angle_rad);

            Mecanum_Set_Velocity(target_speed_x, target_speed_y, 0.0f);

            // 记录最后有效速度与状态
            last_vx = target_speed_x;
            last_vy = target_speed_y;
            lost_cnt = 0; 
            edge_lost_cnt = 0;

            // 记录丢失前的一瞬间，信标是否在视野边缘 (X边界40cm，Y边界70cm)
            is_edge = (fabsf(uart_data[3]) > EDGE_Y);
            if(is_edge){
                target_edge_cnt++;
            }else{
                target_edge_cnt = 0;
            }
        } 
        // ==========================================
        // 状态 2：仅信标 (小车丢失)
        // ==========================================
        else if (locked_state == 2) {
            // 此时无人机算不出小车的相对坐标，只能依靠小车自身的记忆滑行
            
            
            if (lost_cnt <= COAST_CNT) {
                lost_cnt++;
                last_vx *= COAST_DECAY; 
                last_vy *= COAST_DECAY;
                Mecanum_Set_Velocity(last_vx, last_vy, 0.0f); // 【修改点】
            } else {
                Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); // 【修改点】
            }
            target_edge_cnt = 0;
        }
        // ==========================================
        // 状态 1：仅小车 (信标丢失)
        // ==========================================
        else if (locked_state == 1) {
            // 如果信标在边缘丢失，立刻刹车，配合无人机原地扫圈搜索
            if (is_edge) {
                edge_lost_cnt ++;
                if(target_edge_cnt < 5){
                    edge_lost_cnt = EDGE_CNT + 1;
                }
                if(edge_lost_cnt < EDGE_CNT){
                Mecanum_Set_Velocity(last_vx, last_vy, 0.0f);
                }else{
                Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); // 【修改点】
                }
                lost_cnt = COAST_CNT + 1; // 强制超时，防止后续误触发
            } 
            // 如果信标在中心丢失，说明被车底遮挡，滑行 3 帧开出盲区
            else {
                
                if (lost_cnt <= COAST_CNT) {
                    lost_cnt++;
                    last_vx *= COAST_DECAY; 
                    last_vy *= COAST_DECAY;
                    Mecanum_Set_Velocity(last_vx, last_vy, 0.0f); // 【修改点】
                } else {
                    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); // 【修改点】
                }
            }
        }
        // ==========================================
        // 状态 0：全丢 (极度危险)
        // ==========================================
        else {
            // 没有任何参考物，直接强制急停
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); // 【修改点】
            lost_cnt = COAST_CNT + 1; 
            target_edge_cnt = 0;
        }
    }
}

void Mecanum_Control_Loop(void) {

    // 1. 获取反馈速度
    Encoder_GetCount();
    static uint8_t has_unlocked = 0;

    // 检查 IMU 是否校准完毕并解锁底盘
    if(imu_car_rc_data.is_calibrated == 1){
        if(has_unlocked == 0) {
            Mecanum_Unlock();
            has_unlocked = 1; // 标记已解锁，以后不再重复调用
        }
    }else{
        Mecanum_Stop();
        return;
    }

    // ==========================================================
    // 【核心一】只对“用户目标指令”进行斜坡平滑 (防起步打滑)
    // 根据设定的最大加速度，限制每 1ms (CONTROL_DT) 的速度变化量
    // ==========================================================
    if (target_vel.unlock) {
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
    
    if (target_vel.unlock) {
        // 将陀螺仪实际角速度从 deg/s 转换为 rad/s，统一量纲！
        float current_rate_rad = imu_car_rc_data.yaw_rate * (3.1415926f / 180.0f);

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

    if (target_vel.unlock) {
        // 计算原始增量 PID 输出 (此时绝不能限幅)
        motor_output.lf = EN * PID_Calculate_Incremental(&pid_lf, err_lf, CONTROL_DT);
        motor_output.rf = EN * PID_Calculate_Incremental(&pid_rf, err_rf, CONTROL_DT);
        motor_output.lb = EN * PID_Calculate_Incremental(&pid_lb, err_lb, CONTROL_DT);
        motor_output.rb = EN * PID_Calculate_Incremental(&pid_rb, err_rb, CONTROL_DT);

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
    if (target_vel.unlock == true) {
        Motor_Set_Output(MOTOR_LF_PWM, MOTOR_LF_DIR, motor_output.lf);
        Motor_Set_Output(MOTOR_RF_PWM, MOTOR_RF_DIR, motor_output.rf);
        Motor_Set_Output(MOTOR_LB_PWM, MOTOR_LB_DIR, motor_output.lb);
        Motor_Set_Output(MOTOR_RB_PWM, MOTOR_RB_DIR, motor_output.rb);
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