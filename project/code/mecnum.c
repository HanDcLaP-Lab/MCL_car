#include "mecnum.h"

#include "zf_common_headfile.h"
#include <math.h>
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
float dist_out=0;

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
}

// --- [重构] 抽离核心状态为全局，以便底层定时器与急停函数能强制干预 ---
volatile float visual_last_vx = 0.0f;
volatile float visual_last_vy = 0.0f;
volatile uint32_t visual_coast_end_time = 0;  // 目标丢失软滑行绝对物理计时器
volatile uint32_t merge_coast_end_time = 0; // 信标跳变滑行物理计时器
volatile uint8_t  merge_coast_expired = 0;  // merge_coast ISR 到期标志，防竞态清零后跳变检测死循环

extern volatile uint32_t dash_end_time;
extern volatile uint32_t rush_cooldown_end_time;
extern volatile uint8_t dash_source;

void Visual_State_Reset(void) {
    dash_end_time = 0;
    dash_source = 0;
    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    visual_coast_end_time = 0;
    merge_coast_end_time = 0;
    merge_coast_expired = 0;
    // 注意：不在此函数内清零 rush_cooldown_end_time。
    // rush_cooldown 是 dash 到期时由 1ms ISR 设置的 1 秒冷却期，
    // 其目的是防止 0 速度无限重入。若被一并清零，冷却形同虚设，
    // 信标闪烁时 state 1 可无限触发新一轮盲冲。
}

// ================== 底盘使能状态机 (解除武装原因位掩码) ==================
// 上电默认 IMU 未校准 → 锁定。仅当 disarm_flags == 0 时 Chassis_Is_Armed() 为真。
static volatile uint8_t disarm_flags = DISARM_UNCALIBRATED;

// 复位全部 PID 积分项 (解锁起步 / 锁定停车共用)
static void Reset_All_PID(void) {
    PID_Reset(&pid_lf);
    PID_Reset(&pid_rf);
    PID_Reset(&pid_lb);
    PID_Reset(&pid_rb);
    PID_Reset(&pid_yaw_hold);
    PID_Reset(&pid_yaw_rate);
}

// 停车清理：归零目标/平滑速度、直接灭掉 4 路 PWM、复位 PID 与视觉状态
static void Chassis_Apply_Stop(void) {
    smooth_vx = 0.0f;
    smooth_vy = 0.0f;
    smooth_wz = 0.0f;
    Mecanum_Set_Velocity(0, 0, 0);
    pwm_set_duty(MOTOR_LF_PWM, 0);
    pwm_set_duty(MOTOR_RF_PWM, 0);
    pwm_set_duty(MOTOR_LB_PWM, 0);
    pwm_set_duty(MOTOR_RB_PWM, 0);
    Reset_All_PID();
    motor_output.lf = 0; motor_output.rf = 0;
    motor_output.lb = 0; motor_output.rb = 0;
    Visual_State_Reset();
}

bool Chassis_Is_Armed(void) {
    return disarm_flags == 0;
}

uint8_t Chassis_Get_Disarm_Flags(void) {
    return disarm_flags;
}

void Chassis_Block(uint8_t reason) {
    if (disarm_flags & reason) return;          // 该原因已置位，幂等返回 (杜绝1kHz重复清理)
    disarm_flags |= reason;
    Chassis_Apply_Stop();                       // 新增一个解除武装原因 → 确保立即停车清理
}

void Chassis_Unblock(uint8_t reason) {
    if (!(disarm_flags & reason)) return;       // 该原因本就未置位，幂等返回
    disarm_flags &= ~reason;
    if (disarm_flags == 0) {                    // disarmed→armed 跳变：干净起步
        smooth_vx = 0.0f;
        smooth_vy = 0.0f;
        smooth_wz = 0.0f;
        Mecanum_Set_Velocity(0, 0, 0);
        Reset_All_PID();
        Visual_State_Reset();
    }
}
volatile uint32_t sys_time_ms = 0;
volatile uint32_t dash_end_time = 0;          // [重构] 融合盲冲绝对结束物理时间
volatile uint32_t rush_cooldown_end_time = 0; // [重构] 防重入冷却绝对结束时间
volatile uint8_t  dash_source = 0;            // dash 触发来源: 1=状态4融合盲冲, 2=状态1/2单目标丢失盲冲

// ================== 视觉滑行辅助函数 ==================

// 计算跳变检测阈值：prev_car_dist * JUMP_SCALE_COEF 钳位 [JUMP_THRESHOLD_MIN, JUMP_THRESHOLD_MAX] cm
static float Compute_Jump_Threshold(float car_dist) {
    float thr = car_dist * JUMP_SCALE_COEF;
    if (thr < JUMP_THRESHOLD_MIN) thr = JUMP_THRESHOLD_MIN;
    if (thr > JUMP_THRESHOLD_MAX) thr = JUMP_THRESHOLD_MAX;
    return thr;
}

// 统一软滑行处理（含近距恢复检测），替代原先各处盲回放+到期停车模式
// has_prev: 是否有前一帧信标参考位置
// prev_x, prev_y: 参考信标坐标（若检测到近距目标则更新）
// prev_dist: 前一帧车-信标距离
// 返回 1 表示检测到近距目标已恢复并退出 coast
static uint8_t Visual_Coast_With_Recovery(uint8_t has_prev, float *prev_x, float *prev_y, float prev_dist) {
    if (visual_coast_end_time == 0) {
        visual_coast_end_time = sys_time_ms + COAST_HOLD_MS;
    }
    if (sys_time_ms < visual_coast_end_time) {
        // Coast 活动期：检查目标是否回到原信标附近（非跳变），若是则提前退出
        if (has_prev) {
            float dx = uart_data[2] - *prev_x;
            float dy = uart_data[3] - *prev_y;
            float dist_prev = sqrtf(dx * dx + dy * dy);
            if (dist_prev <= Compute_Jump_Threshold(prev_dist)) {
                // 近距：非跳变，退出 coast 接受为原信标
                visual_coast_end_time = 0;
                *prev_x = uart_data[2];
                *prev_y = uart_data[3];
                return 1;
            }
        }
        // 无近距目标或发生跳变：保持滑行
        Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        return 0;
    } else {
        // Coast 到期仍未出现近距目标：接受当前信标（可能是远处另一个信标）
        if (has_prev) {
            *prev_x = uart_data[2];
            *prev_y = uart_data[3];
        }
        visual_coast_end_time = 0;
        Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        return 0;
    }
}

void Visual_Control_Loop(void) {
#if 0
    static uint32_t last_time = 0;
    uint32_t current_time = sys_time_ms;
    uint32_t dt = current_time - last_time;
    last_time = current_time;
    
    // 调试真实执行间隔时可临时打开，常开会干扰控制周期和无线带宽。
    wireless_uart_send_string("dt:");
    wireless_uart_send_int((int32_t)dt);
    wireless_uart_send_string("\r\n");
#endif

    static uint16_t is_edge = 0;
    static uint16_t valid_track_cnt = 0; // [新增] 连续有效跟踪帧数

    if (valid_track_cnt == 0) {
        is_edge = 0; // [隐患修复 P2.10]: 追踪彻底断开时，清空老旧的边缘记忆
    }

    static float prev_target_x = 0.0f, prev_target_y = 0.0f;
    static uint8_t has_prev_target = 0;
    static float prev_car_dist = 0.0f;
    //if(rush_sign) rush_sign = 0; 
    
    if (Chassis_Is_Armed()) {
        // [隐患修复3]: 绝对物理时钟接管系统。一旦盲冲启动，无视后续一切视觉状态强制执行，直到绝对物理时间到达。
        // 这彻底解决了无人机丢包、相机曝光导致单帧时长被放大所引发的冲刺距离失控问题。
        if (dash_end_time > 0) {
            Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
            rush_sign = 1;
            return; // 提前退出，屏蔽后续视觉解析！
        }

        // 0: 全丢, 1: 仅小车, 2: 仅信标, 3: 都有, 4: 发生近距离融合（盲冲）
        uint8_t locked_state = (uint8_t)uart_data[5]; 
        rush_sign = 0;

        if (locked_state == 0) {
            // 仅在全丢时彻底清空历史跟踪记忆；coast 期间保留供跳变检测
            has_prev_target = 0;
            merge_coast_end_time = 0;
        }

        // ==========================================
        // 状态 3：双目标锁定 (正常追踪)
        // ==========================================
        if (locked_state == 3) {
            // 跳变滑行到期标记：到期时跳过跳变检测，直接接受新目标位置，打破死循环。
            // merge_coast_expired 由 1ms ISR 置位，解决 ISR 先于主循环清零 merge_coast_end_time
            // 导致 merge_expired 永不为真的竞态。标志仅一帧有效，消费后即清零。
            uint8_t merge_expired = merge_coast_expired || (merge_coast_end_time > 0 && sys_time_ms >= merge_coast_end_time);
            merge_coast_expired = 0;
            uint8_t in_merge = (merge_coast_end_time > 0 && sys_time_ms < merge_coast_end_time);

            if (in_merge) {
                // 滑行期间：检查新目标是否已回到原信标附近，若回归则退出滑行恢复追踪
                float dx = uart_data[2] - prev_target_x;
                float dy = uart_data[3] - prev_target_y;
                float dist_prev = sqrtf(dx * dx + dy * dy);
                float jump_thr = Compute_Jump_Threshold(prev_car_dist);

                if (dist_prev <= jump_thr) {
                    // 目标回到原信标附近：退出滑行，恢复追踪
                    merge_coast_end_time = 0;
                    in_merge = 0;
                } else {
                    // 仍是远处信标：维持原方向滑行，prev 不更新，死咬原始信标
                    Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
                    if (valid_track_cnt < 1000) valid_track_cnt++;
                    is_edge = (uart_data[7] > EDGE_DIST_CM);
                }
            }

            if (!in_merge) {
                merge_coast_end_time = 0;

                float dist = 0.0f, angle = 0.0f;
                Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

                float car_target_dist = uart_data[7];
                uint8_t jump_detected = 0;

                // merge_coast 到期时跳过跳变检测：直接接受当前信标为新目标
                if (has_prev_target && !merge_expired) {
                    float dx = uart_data[2] - prev_target_x;
                    float dy = uart_data[3] - prev_target_y;
                    float jump = sqrtf(dx * dx + dy * dy);
                    float jump_thr = Compute_Jump_Threshold(prev_car_dist);
                    // [修复] merge_coast 靠重放 visual_last 速度惯性滑过跳变；若信标交接期间速度
                    // 已被 dash/coast 到期的 Visual_State_Reset 清零，则"滑行"会退化为原地死停 400ms
                    // (走一下→停一下→继续走)。无残余速度可滑时直接接受新信标，消除该卡顿。
                    float coast_speed_sq = visual_last_vx * visual_last_vx + visual_last_vy * visual_last_vy;
                    if (jump > jump_thr && coast_speed_sq > 0.01f) {
                        jump_detected = 1;
                        merge_coast_end_time = sys_time_ms + MERGE_COAST_MS;
                    }
                }

                if (jump_detected) {
                    Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
                } else {
                    // 无跳变（含 merge_coast 到期接受新目标）：正常追踪并更新参考坐标
                    
                    dist_out = dist;

                    float current_speed = TARGET_SPEED;

                    float angle_rad = angle * ((float)M_PI / 180.0f);
                    float target_speed_x = current_speed * cosf(angle_rad);
                    float target_speed_y = current_speed * sinf(angle_rad);

                    Mecanum_Set_Velocity(target_speed_x, target_speed_y, 0.0f);

                    visual_last_vx = target_speed_x;
                    visual_last_vy = target_speed_y;

                    prev_target_x = uart_data[2];
                    prev_target_y = uart_data[3];
                    prev_car_dist = car_target_dist;
                    has_prev_target = 1;
                }

                if (valid_track_cnt < 1000) valid_track_cnt++;
                is_edge = (uart_data[7] > EDGE_DIST_CM);
            }
        }
        // ==========================================
        // 状态 4：发生融合，进入盲冲/滑行判断
        // ==========================================
        else if (locked_state == 4) {
            if (!is_edge) {
                uint8_t is_cooldown = (rush_cooldown_end_time > 0 && sys_time_ms < rush_cooldown_end_time);
                // 在中心丢失，极大可能是近距离融合，执行硬实时绝对精确盲冲 (加入1秒防重入冷却)
                if (dash_end_time == 0 && valid_track_cnt > DASH_CNT_THRESHOLD && !is_cooldown) {
                    visual_coast_end_time = 0;
                    float speed_sq = visual_last_vx * visual_last_vx + visual_last_vy * visual_last_vy;
                    float speed = sqrtf(speed_sq);
                    if (speed < 0.1f) speed = 0.1f; // 防除零

                    // 物理绝对时间换算: 时间(s) = 距离(m) / 速度(m/s)
                    float duration_sec = (prev_car_dist / 100.0f) / speed;
                    int32_t duration_ms = (int32_t)(duration_sec * 1000.0f) - 150; // 提前刹车，避免冲过信标

                    if (duration_ms > 600) duration_ms = 600;
                    if (duration_ms < 100) duration_ms = 100;
                    dash_end_time = sys_time_ms + (uint32_t)duration_ms;
                    dash_source = 1;

                    Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
                    rush_sign = 1;
                } else {
                    // 冷却期内或未经历状态3：软滑行（含近距恢复检测）
                    Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist);
                }
            } else {
                // 远处误判融合，软滑行（含近距恢复检测）
                Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist);
            }
        }
        // ==========================================
        // 状态 2 或 1：一方丢失
        // ==========================================
        else if (locked_state == 2 || locked_state == 1) {
            // 单目标丢失时逐步衰减锁定置信度：若信标持续不可见，
            // valid_track_cnt 最终降至 LOCK_THRESHOLD 以下，退出盲冲
            if (valid_track_cnt > 0) valid_track_cnt--;

            if (valid_track_cnt > LOCK_THRESHOLD) {
                // 已锁定：触发不可中断盲冲，维持原方向直到 dash 到期
                visual_coast_end_time = 0;
                uint8_t is_cooldown = (rush_cooldown_end_time > 0 && sys_time_ms < rush_cooldown_end_time);
                if (dash_end_time == 0 && !is_cooldown) {
                    float speed = sqrtf(visual_last_vx * visual_last_vx + visual_last_vy * visual_last_vy);
                    if (speed < 0.1f) speed = 0.1f;
                    uint32_t coast_ms = (uint32_t)(prev_car_dist / 100.0f / speed * 1000.0f);
                    if (coast_ms < DASH_MS_MIN) coast_ms = DASH_MS_MIN;
                    if (coast_ms > DASH_MS_MAX) coast_ms = DASH_MS_MAX;
                    dash_end_time = sys_time_ms + coast_ms;
                    dash_source = 2;
                    Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
                } else if (dash_end_time == 0) {
                    // 冷却期内：软滑行（含近距恢复检测），避免死循环空 dash
                    Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist);
                }
                // 不清 valid_track_cnt：dash 到期后供状态4续用，防止多点亮场景死停
            } else {
                // 未锁定：软滑行（含近距恢复检测）
                Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist);
            }
        }
        // ==========================================
        // 状态 0：全丢
        // ==========================================
        else {
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
            valid_track_cnt = 0;
            dash_end_time = 0;
            visual_coast_end_time = 0;
        }
    }
}

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

    // [最后一道防线] 硬件级绝对时间刹车：如果系统处于盲冲或滑行且绝对时间已到，强行归零指令。
    // 这填补了主循环串口长时间无数据时无法及时刹车的空窗期隐患。
    if (dash_end_time > 0 && sys_time_ms >= dash_end_time) {
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        Visual_State_Reset();
        rush_cooldown_end_time = sys_time_ms + 1000; // [修复] 盲冲结束，强制进入 1 秒冷却，防止 0 速度无限重入
    }
    else if (visual_coast_end_time > 0 && sys_time_ms >= visual_coast_end_time) {
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        Visual_State_Reset();
    }
    else if (merge_coast_end_time > 0 && sys_time_ms >= merge_coast_end_time) {
        // [修复] merge_coast 到期时仅停车，不调 Visual_State_Reset：保留 prev_target/has_prev_target
        // 供 Visual_Control_Loop 恢复后继续追踪（接受新信标），而非清空全部视觉状态
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        merge_coast_expired = 1;    // 置位标志，防 Visual_Control_Loop 竞态重入跳变检测
        merge_coast_end_time = 0;
    }
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
