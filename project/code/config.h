#ifndef _CONFIG_H
#define _CONFIG_H

#define STRATEGY (4)

#define TEST_MODE_NORMAL       0
#define TEST_MODE_TEST         1
#define TEST_MODE_IMU          2
#define TEST_MODE              0

// ================== 航向对齐与地面系解耦开关 ==================
// 1: 启用地面系解耦速度与跨180度最近前/后端航向对齐 (利用前进/后退极速)
// 0: 原有固定0°锁死与车体系速度逻辑
#define HEADING_ALIGN_ENABLE    1

#if STRATEGY == 1
// ================== 小车运动参数 ==================
#define TARGET_SPEED            1.3f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        30.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.4f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 250  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#if STRATEGY == 2
// ================== 小车运动参数 ==================
#define TARGET_SPEED            1.5f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        100.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.6f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 250  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#if STRATEGY == 3
// ================== 小车运动参数 ==================
#define TARGET_SPEED            1.7f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        100.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.6f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 250  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#if STRATEGY == 4
// ================== 小车运动参数 ==================
#define TARGET_SPEED            2.2f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        1000.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  1.0f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 250  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#endif // _CONFIG_H
