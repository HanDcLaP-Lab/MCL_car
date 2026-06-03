#ifndef _CAR_CTRL_H
#define _CAR_CTRL_H

#include "zf_common_headfile.h"

// ===================== 追踪状态机 =====================
// TRACK_IDLE       → 静止等待, 无目标
// TRACK_VALIDATING → 暂态验证 (双条件: 存在性+连续性), 所有新目标必经此状态
// TRACK_ACTIVE     → 正常追踪, 实时跟随目标
// TRACK_COUNTDOWN  → 倒计时滑行, 目标丢失/跳变后保持原方向全速前进
// TRACK_LOST       → 完全丢失, 停车等待新目标验证
typedef enum {
    TRACK_IDLE       = 0,
    TRACK_VALIDATING = 1,
    TRACK_ACTIVE     = 2,
    TRACK_COUNTDOWN  = 3,
    TRACK_LOST       = 4
} CarCtrl_State_t;

// ===================== 功能开关 =====================
#define ENABLE_VALIDATION       1   // [1=强制] 所有新目标必须通过暂态验证, 绝不允许直接跟踪
#define ENABLE_COUNTDOWN        1   // [1=启用] 倒计时滑行: 目标丢失或信标跳变后保持航向全速前进
#define ENABLE_SMOOTH_SWITCH    0   // [0=禁用] 倒计时期间禁止任何目标干扰

// ===================== 追踪速度 (全程恒定, 靠近不减速) =====================
#define TRACK_SPEED_MS          0.6f    // 全程追踪速度 (m/s)

// ===================== 暂态验证: 存在性窗口 (容忍最多30%缺帧) =====================
#define VALIDATE_WINDOW_FRAMES       50    // 滑动窗口大小 (帧, 约1秒@50Hz)
#define VALIDATE_MIN_HITS            35    // 窗口内最少命中帧数 (70% = 容忍30%丢帧)
#define VALIDATE_LOST_TIMEOUT_FRAMES 50   // 连续丢帧超此数退出验证, 0=禁用

// ===================== 暂态验证: 坐标连续性 (同一信标判定) =====================
// 连续两个有效帧之间, 目标原始坐标跳变 ≤ 此值, 超过 = 不同信标 → 重置验证
#define VALIDATE_MAX_JUMP_CM         50.0f // 坐标跳变阈值 (cm)

// ===================== 稳定跟踪确认 (倒计时准入条件) =====================
// ACTIVE 中累计跟踪满此帧数, 才允许目标丢失时进入倒计时滑行
// 未达标就丢失的目标视为不可靠, 直接标记 LOST
#define MIN_ACTIVE_BEFORE_CD_FRAMES  50   // 最少帧数 (约1秒@50Hz)

// ===================== 倒计时参数 =====================
// 倒计时时长: 目标存在时每帧根据距离/车速算出预测到达时间 → 丢失瞬间锁定
// COOLDOWN_MAX_MS 为上限, COUNTDOWN_MIN_MS 为下限
#define COOLDOWN_MAX_MS         3000    // 倒计时上限 (ms, 3秒)
#define COUNTDOWN_MIN_MS        500     // 倒计时下限 (ms)
#define COUNTDOWN_SPEED_DECAY   1.0f    // 速度衰减 (1.0=不变速)
#define ENCODER_MPS_TO_CMPS     100.0f  // m/s → cm/s 换算

// ===================== 目标跳变检测 (信标切换 → 直入倒计时) =====================
// ACTIVE 中目标坐标突然大幅跳变, 判定无人机锁定了另一个信标
// 不跟随新目标, 直入倒计时, 保持原方向全速滑行3秒
#define MERGE_JUMP_MAX_DIST_CM  60.0f   // 仅当旧目标距离 < 此值(cm)时才检测跳变 (远距离噪声大)
#define MERGE_JUMP_CM           50.0f   // 坐标跳变 > 此值(cm)判定为信标切换

// ===================== 边缘检测 =====================
#define EDGE_Y_THRESHOLD        300.0f  // 目标Y坐标 > 此值(cm)判定为视野边缘 (位置不可靠, 快速超时)

// ===================== 接口 =====================
void CarCtrl_Init(void);
void CarCtrl_Update(void);
CarCtrl_State_t CarCtrl_GetState(void);

extern int32_t g_predicted_arrival_ms;

#endif
