#include "chassis_arm.h"
#include "car_image.h"

// ================== 底盘使能状态机 (解除武装原因位掩码) ==================
// 上电默认 IMU 未校准 → 锁定。仅当 disarm_flags == 0 时 Chassis_Is_Armed() 为真。
static volatile uint8_t disarm_flags = DISARM_UNCALIBRATED;

// ================== 内部辅助函数 ==================

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

// ================== 接口函数实现 ==================

bool Chassis_Is_Armed(void) {
    return disarm_flags == 0;
}

uint8_t Chassis_Get_Disarm_Flags(void) {
    return disarm_flags;
}

void Chassis_Block(uint8_t reason) {
    // [CR-21] disarm_flags 读改写加临界区: 主循环(通信/人工)与 1ms ISR(未校准/主循环卡死)
    // 并发 RMW 时避免后写回覆盖对方刚置位的停机位。
    uint32_t primask = interrupt_global_disable();
    if (disarm_flags & reason) {
        interrupt_global_enable(primask);
        return;                                 // 该原因已置位，幂等返回 (杜绝1kHz重复清理)
    }
    disarm_flags |= reason;
    interrupt_global_enable(primask);
    Chassis_Apply_Stop();                       // 新增一个解除武装原因 → 确保立即停车清理
}

void Chassis_Unblock(uint8_t reason) {
    uint32_t primask = interrupt_global_disable();
    if (!(disarm_flags & reason)) {
        interrupt_global_enable(primask);
        return;                                 // 该原因本就未置位，幂等返回
    }
    disarm_flags &= ~reason;
    uint8_t flags_now = disarm_flags;
    interrupt_global_enable(primask);
    if (flags_now == 0) {                       // disarmed→armed 跳变：干净起步
        smooth_vx = 0.0f;
        smooth_vy = 0.0f;
        smooth_wz = 0.0f;
        Mecanum_Set_Velocity(0, 0, 0);
        Reset_All_PID();
        Visual_State_Reset();
    }
}
