#include "mecnum.h"

#include "zf_common_headfile.h"
#include <math.h>
float f_t = 0;

float KP=6000.0f, KI=90000.0f, KD=0.0f, MAX_I=4500.0f;

// ================== 全局变量 ==================
PID_t pid_lf, pid_rf, pid_lb, pid_rb;//速度环pid
PID_t pid_yaw_hold;//角度环pid
PID_t pid_yaw_rate;
// [位置环参数] (暂未启用，位置环控制待实现)
// float POS_KP=0.015f, POS_KI=0.0f, POS_KD=0.0f, POS_MAX_I=0.5f, POS_OUT_MAX=1.0f;

// [参数调整 - 偏航单环PID参数, 已融合角速度内环阻尼到KD] 
float YAW_KP=0.2f, YAW_KI=0.0f, YAW_KD=0.04f, YAW_MAX_I=1.0f, YAW_OUT_MAX=1.75f;

float YAW_RATE_KP=1.5f, YAW_RATE_KI=0.0f, YAW_RATE_KD=0.0f, YAW_RATE_MAX_I=1.0f, YAW_RATE_OUT_MAX=1.5f;
Target_t target_vel = {0};//目标运行情况
Motor_Output_t motor_output = {0};

// [新增] 目标偏航角 (连续累积角度，单位: 度)
float target_yaw = 0.0f;

// [新增] 竞速冲刺对齐速度放大与转角屏蔽参数 (可通过无线串口实时调参)
float ALIGN_SPEED_BOOST = 0.35f;  // 小角度冲刺速度放大系数 (默认放大 1.35 倍)
float SHIELD_TRANS_DEG  = 6.0f;   // 转角屏蔽与加速过渡门限角度 (度)

void Mecanum_Set_Target_Yaw(float yaw) {
    target_yaw = yaw;
}

// 【新增】斜坡函数相关的平滑速度变量
float smooth_vx = 0.0f;
float smooth_vy = 0.0f;
float smooth_wz = 0.0f;
static volatile uint8_t large_turn_accel_state = 0U; // 0:关闭 1:等待新指令 2:过渡中

// [新增] 开环直驱测试状态: 主循环测试写入, 1ms控制ISR读取 (float 32位原子写, M4上无竞态风险)
static volatile float pwm_open_loop_lf = 0.0f;
static volatile float pwm_open_loop_rf = 0.0f;
static volatile float pwm_open_loop_lb = 0.0f;
static volatile float pwm_open_loop_rb = 0.0f;
static volatile uint8_t pwm_open_loop_active = 0U;

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

// [新增] 测试开环直驱: 指定4路PWM占空比并激活 (仅测试模式调用)
void Mecanum_Set_PWM_Open_Loop(float lf, float rf, float lb, float rb) {
    pwm_open_loop_lf = lf;
    pwm_open_loop_rf = rf;
    pwm_open_loop_lb = lb;
    pwm_open_loop_rb = rb;
    pwm_open_loop_active = 1U;
}

// [新增] 关闭开环直驱并清零存储占空比 (防止陈旧占空比复活泄漏)
void Mecanum_Set_PWM_Open_Loop_Off(void) {
    pwm_open_loop_active = 0U;
    pwm_open_loop_lf = 0.0f;
    pwm_open_loop_rf = 0.0f;
    pwm_open_loop_lb = 0.0f;
    pwm_open_loop_rb = 0.0f;
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
    target_yaw = 0.0f;
}

volatile uint32_t sys_time_ms = 0;
volatile uint32_t main_loop_heartbeat_ms = 0;   // 主循环存活心跳 (由 main_cm4 主循环刷新)

void Mecanum_Control_Loop(void) {

    // 1. 获取反馈速度
    Encoder_GetCount();

    // 检查 IMU 是否校准完毕：未校准则锁定底盘并退出。
    // Block/Unblock 幂等，校准期间每 tick 调用也不会重复执行停车清理。
    if(imu_car_data.is_calibrated == 1){
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
    // [修复] 测试模式已并入主循环(每轮喂心跳)，同样受看门狗保护；仅 IMU 模式
    // 自带独立 while(1) 不喂心跳，维持关闭。
    if (TEST_MODE != TEST_MODE_IMU && ((uint32_t)(sys_time_ms - main_loop_heartbeat_ms) > MAINLOOP_STALL_MS)) {
        Chassis_Block(DISARM_MAINLOOP_STALL);    // 幂等; 仅置位跳变时执行一次停车清理
    }
    uint8_t armed = Chassis_Is_Armed();

    // ISR 级时间刹车检查 (dash 到期处理)，由 car_image.c 实现
    Visual_Brake_Check();

    // [新增] 测试开环直驱: 跳过斜坡平滑/偏航串级/轮速PID, 直接输出指定PWM占空比。
    // 仅在 armed 时生效; 未解锁(急停/断连)时落入下方 !armed 分支灭PWM, 急停安全不受影响。
    if (armed && pwm_open_loop_active) {
        Motor_Set_Output(MOTOR_LF_PWM, MOTOR_LF_DIR, pwm_open_loop_lf);
        Motor_Set_Output(MOTOR_RF_PWM, MOTOR_RF_DIR, pwm_open_loop_rf);
        Motor_Set_Output(MOTOR_LB_PWM, MOTOR_LB_DIR, pwm_open_loop_lb);
        Motor_Set_Output(MOTOR_RB_PWM, MOTOR_RB_DIR, pwm_open_loop_rb);
        return;
    }
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
    // 【核心二】偏航单环 PID (死死咬住航向，带局部连续转角屏蔽)
    // ==========================================================
    float final_wz = smooth_wz; // 默认采用平滑后的目标自转速度
    float yaw_error = 0.0f;
    
    if (armed) {
        // 如果外部没有要求自转 (判断平滑后的 smooth_wz 近似为0)，启动 Yaw 锁死
        if (fabsf(smooth_wz) < 0.05f) {
            
            // --- 角度环控制 (直接输出期望底盘自转角速度 rad/s) ---
#if HEADING_ALIGN_ENABLE
            yaw_error = target_yaw - imu_car_data.yaw_total;   // 单位：度 (动态最近对准目标航向)
#else
            yaw_error = 0.0f - imu_car_data.yaw_total;         // 单位：度 (固定0°锁定)
#endif

            // [新增] 局部连续转角屏蔽权重: 仅在门限内平滑衰减，门限外保持 100% 满额转向力矩
            float abs_err = fabsf(yaw_error);
            float yaw_weight = 1.0f;
            if (SHIELD_TRANS_DEG > 0.001f && abs_err < SHIELD_TRANS_DEG) {
                // 升余弦平滑函数 (C1连续，端点导数为0，彻底杜绝跳变与临界振荡)
                yaw_weight = 0.5f * (1.0f - cosf(abs_err / SHIELD_TRANS_DEG * (float)M_PI));
            }

            // 单环 PID 直接输出底盘目标自转角速度 final_wz (rad/s)
            final_wz = PID_Calculate(&pid_yaw_hold, yaw_error, CONTROL_DT);
            final_wz *= yaw_weight; // 小角度平滑闭锁转角，中大角度 100% 满额自转

        } else {
            // 如果外部发送了主动旋转命令，复位角度环并直接跟踪平滑角速度
            PID_Reset(&pid_yaw_hold); 
            target_yaw = imu_car_data.yaw_total;
            final_wz = smooth_wz;
        }
    }
    f_t = final_wz; // 记录用于调试输出

    // ==========================================================
    // 【核心三】运动学逆解算 (对齐提速放大 + 屏蔽横移)
    // ==========================================================
#if HEADING_ALIGN_ENABLE
    // 将地面系平滑速度实时投影到当前车体系 (X前, Y左)
    float cur_yaw_rad = imu_car_data.yaw * ((float)M_PI / 180.0f);
    float cos_yaw = cosf(cur_yaw_rad);
    float sin_yaw = sinf(cur_yaw_rad);
    float body_vx = smooth_vx * cos_yaw + smooth_vy * sin_yaw;
    float body_vy = smooth_vx * sin_yaw - smooth_vy * cos_yaw;

    // [新增] 小角度冲刺速度连续放大与横移平滑消除 (无中段动力凹陷)
    float abs_err = fabsf(yaw_error);
    if (SHIELD_TRANS_DEG > 0.001f && abs_err < SHIELD_TRANS_DEG) {
        float boost_weight = 0.5f * (1.0f + cosf(abs_err / SHIELD_TRANS_DEG * (float)M_PI));
        float speed_scale = 1.0f + ALIGN_SPEED_BOOST * boost_weight;
        body_vx *= speed_scale;
        body_vy *= (1.0f - boost_weight); // 小角度平滑消除横移损耗
    }
#else
    float body_vx = smooth_vx;
    float body_vy = smooth_vy;
#endif

    // 使用闭环输出的 final_wz 直接计算旋转所需的差速
    float center_v = final_wz * (CAR_L + CAR_W);

    // 逆解算公式
    target_vel.v_lf = body_vx - body_vy + center_v;
    target_vel.v_rf = body_vx + body_vy - center_v;

    target_vel.v_lb = body_vx + body_vy + center_v;
    target_vel.v_rb = body_vx - body_vy - center_v;

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
