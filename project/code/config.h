#ifndef _CONFIG_H
#define _CONFIG_H

// ================== 小车运动参数 ==================
#define TARGET_SPEED            0.88f   // 目标速度 (m/s)
#define MAX_ACCEL_LINEAR        5.0f   // 平移加速度上限 (m/s^2)
#define LARGE_TURN_ACCEL_SCALE  0.3f   // 大角度换向时的平移加速度倍率
#define MAX_ACCEL_W             1.0f   // 旋转加速度上限 (rad/s^2)

// ================== Dash 参数 ==================
#define POST_DASH_HOLD_MS       0U    // 完全刹停后静止等待视觉稳定 (ms)

#endif // _CONFIG_H
