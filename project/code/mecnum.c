#include "mecnum.h"

#include "zf_common_headfile.h"
#include <math.h>
// [新增] volatile: EN 在 ISR (Mecanum_Control_Loop) 和主循环之间共享，必须 volatile 防止编译器缓存
volatile int EN = 1;
float f_t = 0;
int rush_sign = 0;
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
    PID_Reset(&pid_yaw_rate);
}
volatile uint32_t sys_time_ms = 0;
volatile uint32_t dash_end_time = 0;          // [重构] 融合盲冲绝对结束物理时间
volatile uint32_t rush_cooldown_end_time = 0; // [重构] 防重入冷却绝对结束时间

void Visual_Control_Loop(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = sys_time_ms;
    uint32_t dt = current_time - last_time;
    last_time = current_time;
    
    // 无线打印真实执行间隔
    wireless_uart_send_string("dt:");
    wireless_uart_send_int((int32_t)dt);
    wireless_uart_send_string("\r\n");

    static uint16_t lost_cnt = 0;
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    static uint16_t is_edge = 0;
    static uint16_t valid_track_cnt = 0; // [新增] 连续有效跟踪帧数

    static float prev_target_x = 0.0f, prev_target_y = 0.0f;
    static uint8_t has_prev_target = 0;
    static uint32_t merge_coast_end_time = 0; // [重构] 跳变滑行物理时间
    static float prev_car_dist = 0.0f;
    //if(rush_sign) rush_sign = 0; 
    
    if (target_vel.unlock) {
        // [隐患修复3]: 绝对物理时钟接管系统。一旦盲冲启动，无视后续一切视觉状态强制执行，直到绝对物理时间到达。
        // 这彻底解决了无人机丢包、相机曝光导致单帧时长被放大所引发的冲刺距离失控问题。
        if (dash_end_time > 0) {
            if (sys_time_ms >= dash_end_time) {
                Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); // 冲刺物理时间到达，精准刹车
                
                // [隐患修复5]: 彻底清空遗留速度状态！
                last_vx = 0.0f;
                last_vy = 0.0f;
                lost_cnt = COAST_CNT + 1; // 让普通的滑行计时器直接过期
                valid_track_cnt = 0;
                dash_end_time = 0;
                rush_cooldown_end_time = sys_time_ms + 1000; // 开启绝对物理 1秒 冷却，防止连冲
            } else {
                Mecanum_Set_Velocity(last_vx, last_vy, 0.0f);
                rush_sign = 1;
            }
            return; // 提前退出，屏蔽后续视觉解析！
        }

        // 0: 全丢, 1: 仅小车, 2: 仅信标, 3: 都有, 4: 发生近距离融合（盲冲）
        uint8_t locked_state = (uint8_t)uart_data[5]; 
        rush_sign = 0;

        // [隐患修复] 一旦丢失双目标锁定（进入单目标丢失、全丢、或融合盲冲），
        // 必须立刻清空“上一个目标的记忆”。防止在远处重新点亮信标时，因坐标突变引发漫长的防抖滑行。
        if (locked_state != 3) {
            has_prev_target = 0;
            merge_coast_end_time = 0;
        }

        // ==========================================
        // 状态 3：双目标锁定 (正常追踪)
        // ==========================================
        if (locked_state == 3) {
            float dist = 0.0f, angle = 0.0f;
            Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

            // 跳变检测：距离近 + target 坐标突变 → 用旧方向滑行
            float car_target_dist = uart_data[7];
            if (has_prev_target && prev_car_dist < MERGE_DIST_THRESHOLD) {
                float dx = uart_data[2] - prev_target_x;
                float dy = uart_data[3] - prev_target_y;
                if (sqrtf(dx * dx + dy * dy) > MERGE_JUMP_THRESHOLD) {
                    merge_coast_end_time = sys_time_ms + 400; // 绝对物理过滤 400 毫秒跳变
                }
            }
            prev_target_x = uart_data[2];
            prev_target_y = uart_data[3];
            prev_car_dist = car_target_dist;
            has_prev_target = 1;

            if (merge_coast_end_time > 0 && sys_time_ms < merge_coast_end_time) {
                Mecanum_Set_Velocity(last_vx, last_vy, 0.0f); // 在此期间保持上一次的速度
            } else {
                merge_coast_end_time = 0;
                ang_out = angle;
                dist_out = dist;

                float current_speed = TARGET_SPEED;
                // if (dist < 20.0f) current_speed = 0.35f; 

                float angle_rad = angle * ((float)M_PI / 180.0f);
                float target_speed_x = current_speed * cosf(angle_rad);
                float target_speed_y = current_speed * sinf(angle_rad);

                Mecanum_Set_Velocity(target_speed_x, target_speed_y, 0.0f);

                last_vx = target_speed_x;
                last_vy = target_speed_y;
            }

            lost_cnt = 0;
            dash_end_time = 0;
            if (valid_track_cnt < 1000) valid_track_cnt++; // 累积有效帧
            
            // 判断小车与信标之间的相对距离是否超过 200cm
            is_edge = (uart_data[7] > 200.0f);
        } 
        // ==========================================
        // [新增] 状态 4：发生融合，进入盲冲/滑行判断
        // ==========================================
        else if (locked_state == 4) {
            if (!is_edge) {
                uint8_t is_cooldown = (rush_cooldown_end_time > 0 && sys_time_ms < rush_cooldown_end_time);
                // 在中心丢失，极大可能是近距离融合，执行硬实时绝对精确盲冲 (加入1秒防重入冷却)
                if (dash_end_time == 0 && valid_track_cnt > 0 && !is_cooldown) {
                    float speed_sq = last_vx * last_vx + last_vy * last_vy;
                    float speed = sqrtf(speed_sq);
                    if (speed < 0.1f) speed = 0.1f; // 防除零
                    
                    // 物理绝对时间换算: 时间(s) = 距离(m) / 速度(m/s)
                    float duration_sec = (prev_car_dist / 100.0f) / speed;
                    uint32_t duration_ms = (uint32_t)(duration_sec * 1000.0f) - 150; // 追加 200ms 余量确保越过信标
                    
                    // [隐患修复4]: 限制盲冲最高物理时间为 800 毫秒，防止算出天文数字失控
                    if (duration_ms > 600) duration_ms = 600;
                    if (duration_ms < 100) duration_ms = 100;
                    // 挂载绝对硬实时物理定时器
                    dash_end_time = sys_time_ms + duration_ms;
                    
                    // 第一帧立马执行
                    Mecanum_Set_Velocity(last_vx, last_vy, 0.0f);
                    rush_sign = 1;
                } else if (dash_end_time == 0) {
                    // [指令真空填补]: 突兀的跳变或者不合法的融合（如冷却期内或未经历状态3）
                    // 绝不信任该孤立噪点，立刻停车保平安，并清空可能越界的累积
                    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
                    valid_track_cnt = 0;
                }
            } else {
                // 在边缘误判融合，按边缘防闪烁逻辑处理
                if (valid_track_cnt > 10 && lost_cnt < 5) {
                    lost_cnt++;
                    last_vx *= COAST_DECAY; 
                    last_vy *= COAST_DECAY;
                    Mecanum_Set_Velocity(last_vx, last_vy, 0.0f);
                } else {
                    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
                    valid_track_cnt = 0;
                    lost_cnt = COAST_CNT + 1; // 强制过期，防止停滞
                }
            }
        }
        // ==========================================
        // 状态 2 或 1：一方丢失 (应用边缘防闪烁过滤)
        // ==========================================
        else if (locked_state == 2 || locked_state == 1) {
            if (is_edge) {
                // 边缘丢失防闪烁: 只有稳定跟踪后才允许滑行5帧，否则立刻刹车
                if (valid_track_cnt > 10 && lost_cnt < 5) {
                    lost_cnt++;
                    last_vx *= COAST_DECAY; 
                    last_vy *= COAST_DECAY;
                    Mecanum_Set_Velocity(last_vx, last_vy, 0.0f);
                } else {
                    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
                    if (lost_cnt >= 5) valid_track_cnt = 0;
                }
            } else {
                // 中心常规丢失，执行原有的衰减逻辑
                if (lost_cnt <= COAST_CNT) {
                    lost_cnt++;
                    last_vx *= COAST_DECAY; 
                    last_vy *= COAST_DECAY;
                    Mecanum_Set_Velocity(last_vx, last_vy, 0.0f); 
                } else {
                    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); 
                    valid_track_cnt = 0;
                }
            }
        }
        // ==========================================
        // 状态 0：全丢
        // ==========================================
        else {
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f); 
            lost_cnt = COAST_CNT + 1; 
            valid_track_cnt = 0;
            dash_end_time = 0;
        }
    }
}

void Mecanum_Control_Loop(void) {
    sys_time_ms++;

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

    // [最后一道防线] 硬件级绝对时间刹车：如果系统处于盲冲且绝对时间已到，强行归零指令。
    // 这填补了主循环串口长时间无数据时无法及时刹车的空窗期隐患。
    if (dash_end_time > 0 && sys_time_ms >= dash_end_time) {
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
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