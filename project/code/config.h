#ifndef _CONFIG_H
#define _CONFIG_H

#define STRATEGY (3)

#if STRATEGY == 1
// ================== 小车运动参数 ==================
#define TARGET_SPEED            0.7f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        5.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.3f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 100  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#if STRATEGY == 2
// ================== 小车运动参数 ==================
#define TARGET_SPEED            0.88f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        7.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.3f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 400  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#if STRATEGY == 3
// ================== 小车运动参数 ==================
#define TARGET_SPEED            0.95f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        7.5f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.2f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define DASH_DURATION_COMPENSATION_MS 400  // 计算时长限幅后的有符号补偿，可正可负 (ms)
#define POST_DASH_HOLD_MS             0U   // 完全刹停后静止等待视觉稳定 (ms)
#endif

#endif // _CONFIG_H
