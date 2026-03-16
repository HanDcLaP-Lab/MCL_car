#include "mecnum.h"

#include "zf_common_headfile.h"
#include <math.h>
int EN = 1;
// [参数调整] 针对增量式PID (dt=0.001s) 的调优参数
// KP=3500: 0.5m/s 误差时提供 1750 的基础PWM，确保启动有力
// KI=3000: 0.5m/s 误差时每秒增加 1500 PWM (3000*0.5*0.001*1000)，消除静差只需约0.5-1秒
// KD=0: 速度环通常不需要微分项，除非超调严重
float KP=1500.0f, KI=5400.0f, KD=0.0f, MAX_I=3500.0f;

// ================== 全局变量 ==================
PID_t pid_lf, pid_rf, pid_lb, pid_rb;//速度环pid
PID_t pid_yaw_hold;//角度环pid
PID_t pid_pos_x, pid_pos_y; // 位置环pid
// [位置环参数] 1cm误差对应0.015m/s速度
float POS_KP=0.015f, POS_KI=0.0f, POS_KD=0.0f, POS_MAX_I=0.5f, POS_OUT_MAX=1.0f;

// [参数调整] 
float YAW_KP=0.075f, YAW_KI=0.0f, YAW_KD=0.001f, YAW_MAX_I=0.01f, YAW_OUT_MAX=4.0f;
Target_t target_vel = {0};//目标运行情况
Motor_Output_t motor_output = {0};
float ang_out=0,dist_out=0;

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
    PID_Init(&pid_yaw_hold, YAW_KP, YAW_KI, YAW_KD, YAW_MAX_I, YAW_OUT_MAX);
    PID_Init(&pid_pos_x, POS_KP, POS_KI, POS_KD, POS_MAX_I, POS_OUT_MAX);
    PID_Init(&pid_pos_y, POS_KP, POS_KI, POS_KD, POS_MAX_I, POS_OUT_MAX);

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
    PID_Reset(&pid_yaw_hold);
    PID_Reset(&pid_pos_x);
    PID_Reset(&pid_pos_y);

    motor_output.lf = 0; motor_output.rf = 0;
    motor_output.lb = 0; motor_output.rb = 0;
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
    PID_Reset(&pid_yaw_hold);
    PID_Reset(&pid_pos_x);
    PID_Reset(&pid_pos_y);
}
void Visual_Control_Loop(void) {
    static uint8_t lost_cnt = 0;
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    static uint8_t is_edge = 0;

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
            Image_Solve(imu_car_data.yaw, &dist, &angle);

            ang_out = angle;
            dist_out = dist;

            float current_speed = TARGET_SPEED;
            if (dist < 8.0f) { 
                current_speed = 0.0f; // 死区刹车
            } else if (dist < 35.0f) {
                current_speed = TARGET_SPEED * (dist / 35.0f); // 比例减速
                if (current_speed < 0.15f) current_speed = 0.15f; 
            }

            float angle_rad = angle * (float)(3.1415926f / 180.0f);
            float target_speed_x = current_speed * cosf(angle_rad);
            float target_speed_y = current_speed * sinf(angle_rad);

            Mecanum_Set_Velocity(target_speed_x, target_speed_y, 0.0f);

            // 记录最后有效速度与状态
            last_vx = target_speed_x;
            last_vy = target_speed_y;
            lost_cnt = 0; 

            // 记录丢失前的一瞬间，信标是否在视野边缘 (X边界40cm，Y边界70cm)
            is_edge = (fabsf(uart_data[2]) > EDGE_X) || (fabsf(uart_data[3]) > EDGE_Y);
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
        }
        // ==========================================
        // 状态 1：仅小车 (信标丢失)
        // ==========================================
        else if (locked_state == 1) {
            // 如果信标在边缘丢失，立刻刹车，配合无人机原地扫圈搜索
            if (is_edge) {
                Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); // 【修改点】
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
        }
    }
}

void Mecanum_Control_Loop(void) {

    // 1. 获取反馈速度
    Encoder_GetCount();
    static uint8_t has_unlocked = 0;

    if(imu_car_data.is_calibrated == 1){
        if(has_unlocked == 0) {
            Mecanum_Unlock();
            has_unlocked = 1; // 标记已解锁，以后不再重复调用
        }
    }else{
        Mecanum_Stop();
        return;
    }
    // 2. Yaw角闭环 (周期 1ms)
    float final_wz = target_vel.wz; // 默认采用外部设定的期望自转速度
    if (target_vel.unlock) {
        // 判断：如果外部没有要求自转(wz近似为0)，则启动 Yaw 闭环锁死车头
        if (fabsf(target_vel.wz) < 0.05f) {
            // 目标永远指向0度
            float yaw_error = 0.0f - imu_car_data.yaw_total; 
            
            // 这里的 PID 输出直接充当最终的旋转角速度
            final_wz = PID_Calculate(&pid_yaw_hold, yaw_error, CONTROL_DT);
        } else {
            // 如果外部有旋转命令(如测试程序)，暂停角度环，防止互相打架
            PID_Reset(&pid_yaw_hold); 
        }
    }
    // 3. 运动学逆解算 (Inverse Kinematics)
    // 根据车身坐标系 V_x, V_y, Omega 计算四个轮子的线速度
    // 注意：这里的正负号取决于电机安装方向和轮子类型 (A/B轮布局)
    // 典型布局：左前/右后为A轮，右前/左后为B轮
    float center_v = final_wz * (CAR_L + CAR_W);

    target_vel.v_lf = target_vel.vx - target_vel.vy - center_v;
    target_vel.v_rf = target_vel.vx + target_vel.vy + center_v;
    target_vel.v_lb = target_vel.vx + target_vel.vy - center_v;
    target_vel.v_rb = target_vel.vx - target_vel.vy + center_v;

    
    // 0.055 是 1ms 下 3200线编码器的最小分辨率
    float err_lf = target_vel.v_lf - encoder_data.lf;
    float err_rf = target_vel.v_rf - encoder_data.rf;
    float err_lb = target_vel.v_lb - encoder_data.lb;
    float err_rb = target_vel.v_rb - encoder_data.rb;

    // 3. PID 计算
    if (target_vel.unlock) {
        motor_output.lf = EN * PID_Calculate_Incremental(&pid_lf, err_lf, CONTROL_DT);
        motor_output.rf = EN * PID_Calculate_Incremental(&pid_rf, err_rf, CONTROL_DT);
        motor_output.lb = EN * PID_Calculate_Incremental(&pid_lb, err_lb, CONTROL_DT);
        motor_output.rb = EN * PID_Calculate_Incremental(&pid_rb, err_rb, CONTROL_DT);

        // 简单的误差死区处理，防止静止时电机抖动
        // 误差死区仅在目标速度为0时启用，防止运动中输出被锁死在当前值
        // 如果在运动中强制 err=0，增量式PID会保持当前的高PWM输出，导致无法减速
        if (fabsf(target_vel.v_lf) < 0.01f && fabsf(err_lf) < 0.03f) motor_output.lf = 0;
        if (fabsf(target_vel.v_rf) < 0.01f && fabsf(err_rf) < 0.03f) motor_output.rf = 0;
        if (fabsf(target_vel.v_lb) < 0.01f && fabsf(err_lb) < 0.03f) motor_output.lb = 0;
        if (fabsf(target_vel.v_rb) < 0.01f && fabsf(err_rb) < 0.03f) motor_output.rb = 0;
    } else {
        motor_output.lf = 0;
        motor_output.rf = 0;
        motor_output.lb = 0;
        motor_output.rb = 0;
    }

    // 4. 执行电机控制
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