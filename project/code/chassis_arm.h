#ifndef _CHASSIS_ARM_H
#define _CHASSIS_ARM_H

#include "zf_common_headfile.h"

// ================== 底盘使能状态 (解除武装原因位掩码) ==================
// 小车仅在 disarm_flags == 0 (所有原因都清零) 时才允许输出动力。
// 每个来源只置/清自己那一位，互不干扰；人工急停与通讯丢失彼此独立，
// 重连只清通讯位，不会覆盖人工急停。
#define DISARM_UNCALIBRATED  (1u << 0)   // IMU 未校准完成
#define DISARM_COMM_LOST     (1u << 1)   // 无人机通讯看门狗超时 (按字节到达时刻判定)
#define DISARM_MANUAL        (1u << 2)   // 无线通道8人工急停
#define DISARM_DRONE_STOPPED (1u << 3)   // 无人机下传 car_en=0
#define DISARM_MAINLOOP_STALL (1u << 4)  // 主循环卡死超时 (锁存; 仅新到达的合法帧可解除, 见 main_cm4.c)

// ================== 函数声明 ==================
/**
 * @brief 底盘使能状态控制 (解除武装原因位掩码)
 * @note  Block/Unblock 幂等：位状态未变化时直接返回，不重复执行清理。
 *        armed→disarmed 跳变时停车清理；disarmed→armed 跳变时复位 PID 与视觉状态。
 */
bool Chassis_Is_Armed(void);                 // 所有原因清零 → 已武装(允许动)
void Chassis_Block(uint8_t reason);          // 置位某个解除武装原因
void Chassis_Unblock(uint8_t reason);        // 清除某个解除武装原因
uint8_t Chassis_Get_Disarm_Flags(void);      // 读取当前位掩码 (调试用)

#endif // _CHASSIS_ARM_H
