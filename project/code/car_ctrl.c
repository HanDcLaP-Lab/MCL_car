#include "car_ctrl.h"
#include "mecnum.h"
#include "car_image.h"
#include "encoder.h"
#include "imu_car_rc.h"
#include <math.h>
#include <string.h>

// ===================== 外部引用 =====================
extern volatile int EN;
extern int32_t cnt;          // 1ms tick from cm4_isr.c

// ===================== 模块静态状态 =====================
static CarCtrl_State_t g_state = TRACK_IDLE;

// 暂态验证：存在性滑动窗口 (容忍最多30%缺帧)
static uint8_t g_val_win[VALIDATE_WINDOW_FRAMES];
static uint8_t g_val_idx;
static uint8_t g_val_hits;

// 暂态验证：坐标连续性追踪 (同一信标判定)
// 记录上一个有效帧的目标原始坐标, 用于检测坐标跳变
static float  g_val_last_tx, g_val_last_ty;
static uint8_t g_val_has_prev_pos;

// 暂态验证：连续丢帧计数器 (进入 VALIDATING 时重置)
static uint16_t g_val_lost;

// 倒计时
static int32_t g_cd_end_ms;
static float   g_cd_vx_cm, g_cd_vy_cm;

// 上一次速度和方向 (用于倒计时/跳变时保持航向)
static float g_last_vx, g_last_vy;

// 边缘
static uint16_t g_edge_cnt;
static uint8_t  g_is_edge;

// 跳变检测 (用原始 target_x/y)
static float g_prev_tx_raw, g_prev_ty_raw;
static float g_prev_car_dist;
static uint8_t g_has_prev;

// 稳定跟踪帧数: 进入 ACTIVE 后累计, 用于倒计时准入判定
// 只有累计帧数达标的目标才被信任为"稳定存在的信标"
static uint16_t g_active_frames;

// 调试：目标存在时每帧更新的预测到达时间, 丢失瞬间锁定为倒计时时长
int32_t g_predicted_arrival_ms = 0;

// ===================== 内部函数 =====================

// 目标存在时调用: 根据当前距离和车速计算还需多少ms到达
static int32_t ComputeArrivalMs(float dist_cm) {
    float avg_spd_cm = (fabsf(encoder_data.lf) + fabsf(encoder_data.rf) +
                        fabsf(encoder_data.lb) + fabsf(encoder_data.rb))
                       * 0.25f * ENCODER_MPS_TO_CMPS;
    if (avg_spd_cm < 1.0f) return COOLDOWN_MAX_MS;
    int32_t est = (int32_t)(dist_cm / avg_spd_cm * 1000.0f);
    if (est > COOLDOWN_MAX_MS) est = COOLDOWN_MAX_MS;
    if (est < COUNTDOWN_MIN_MS) est = COUNTDOWN_MIN_MS;
    return est;
}

// ===================== 接口函数 =====================

void CarCtrl_Init(void) {
    g_state = TRACK_IDLE;
    g_val_idx = 0;
    g_val_hits = 0;
    g_val_has_prev_pos = 0;
    g_val_lost = 0;
    memset(g_val_win, 0, sizeof(g_val_win));
    g_cd_end_ms = 0;
    g_cd_vx_cm = 0.0f;
    g_cd_vy_cm = 0.0f;
    g_last_vx = 0.0f;
    g_last_vy = 0.0f;
    g_edge_cnt = 0;
    g_is_edge = 0;
    g_prev_tx_raw = 0.0f;
    g_prev_ty_raw = 0.0f;
    g_prev_car_dist = 0.0f;
    g_has_prev = 0;
    g_active_frames = 0;
    g_predicted_arrival_ms = 0;
}

CarCtrl_State_t CarCtrl_GetState(void) {
    return g_state;
}

void CarCtrl_Update(void) {
    if (!target_vel.unlock) {
        CarCtrl_Init();
        return;
    }

    uint8_t locked_state = (uint8_t)(car.locked_state + 0.5f);
    uint8_t target_present = (locked_state == 3);

    switch (g_state) {

    case TRACK_IDLE:
        if (target_present) {
            // 所有新目标必须通过暂态验证 (绝不允许直接跟踪)
            memset(g_val_win, 0, sizeof(g_val_win));
            g_val_idx = 0;
            g_val_hits = 0;
            g_val_has_prev_pos = 0;
            g_val_lost = 0;
            g_state = TRACK_VALIDATING;
        } else {
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        }
        break;

    case TRACK_VALIDATING:
    {
        // ================================================================
        // 双条件暂态验证: 同时满足"存在性"和"连续性"才采信目标
        //
        // 条件1 (存在性) — 滑动窗口: 容忍最多30%缺帧
        //   VALIDATE_WINDOW_FRAMES 帧窗口内, 至少 VALIDATE_MIN_HITS 帧有目标
        //
        // 条件2 (连续性) — 坐标跳变检测: 确保始终是同一个信标
        //   连续两个有效帧的目标原始坐标跳变 ≤ VALIDATE_MAX_JUMP_CM
        //   若跳变超限 → 判定为不同信标 → 重置全部验证状态
        // ================================================================

        // --- 连续丢帧超时 → 放弃验证, 回 IDLE ---
        if (!target_present) {
            g_val_lost++;
#if VALIDATE_LOST_TIMEOUT_FRAMES > 0
            if (g_val_lost > VALIDATE_LOST_TIMEOUT_FRAMES) {
                g_state = TRACK_IDLE;
                Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
                g_val_lost = 0;
                g_val_has_prev_pos = 0;
                break;
            }
#endif
        } else {
            g_val_lost = 0;
        }

        // --- 条件2: 坐标连续性检查 (同一信标判定) ---
        uint8_t position_ok = 1;
        if (target_present) {
            if (g_val_has_prev_pos) {
                float dx = car.target_x - g_val_last_tx;
                float dy = car.target_y - g_val_last_ty;
                float jump = sqrtf(dx * dx + dy * dy);
                position_ok = (jump <= VALIDATE_MAX_JUMP_CM);
            }
            // 首次有效帧: 无条件通过, 记录坐标作为后续比较基准
            if (position_ok) {
                g_val_last_tx = car.target_x;
                g_val_last_ty = car.target_y;
                g_val_has_prev_pos = 1;
            }
        }

        // 坐标不连续 → 判定为不同信标 → 重置验证
        if (!position_ok) {
            memset(g_val_win, 0, sizeof(g_val_win));
            g_val_idx = 0;
            g_val_hits = 0;
            g_val_has_prev_pos = 0;
        }

        // --- 条件1: 滑动窗口更新 (仅在坐标连续时写入, 防止不同信标的帧污染窗口) ---
        if (position_ok) {
            uint8_t old = g_val_win[g_val_idx];
            g_val_win[g_val_idx] = target_present ? 1 : 0;
            if (target_present && !old) g_val_hits++;
            if (!target_present && old) g_val_hits--;
            g_val_idx = (g_val_idx + 1) % VALIDATE_WINDOW_FRAMES;
        }

        // --- 双条件同时满足 → 进入 ACTIVE 追踪 ---
        if (g_val_hits >= VALIDATE_MIN_HITS) {
            g_state = TRACK_ACTIVE;
            g_active_frames = 0;
            g_edge_cnt = 0;
            g_is_edge = 0;
            g_has_prev = 0;
        }

        // --- 运动控制: 全速 0.6m/s, 缺帧也不减速 ---
        if (target_present) {
            float dist = 0.0f, angle = 0.0f;
            Image_Solve(imu_car_rc_data.yaw, &dist, &angle);
            float rad = angle * ((float)M_PI / 180.0f);
            g_last_vx = TRACK_SPEED_MS * cosf(rad);
            g_last_vy = TRACK_SPEED_MS * sinf(rad);
            Mecanum_Set_Velocity(g_last_vx, g_last_vy, 0.0f);
        } else {
            Mecanum_Set_Velocity(g_last_vx, g_last_vy, 0.0f);
        }
        break;
    }

    case TRACK_ACTIVE:
    {
        g_active_frames++;

        // --- 目标丢失 → 用上一帧(目标还在时)算好的预测时间 ---
        if (!target_present) {
            if (g_active_frames >= MIN_ACTIVE_BEFORE_CD_FRAMES) {
                g_cd_end_ms = cnt + g_predicted_arrival_ms;
                g_cd_vx_cm = g_last_vx * ENCODER_MPS_TO_CMPS;
                g_cd_vy_cm = g_last_vy * ENCODER_MPS_TO_CMPS;
                g_state = TRACK_COUNTDOWN;
            } else {
                g_state = TRACK_LOST;
            }
            break;
        }

        // --- 目标存在: 解算距离角度, 更新预测, 跳变检测 ---
        float dist = 0.0f, angle = 0.0f;
        Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

        g_predicted_arrival_ms = ComputeArrivalMs(dist);

        if (g_has_prev && g_prev_car_dist < MERGE_JUMP_MAX_DIST_CM) {
            float dx = car.target_x - g_prev_tx_raw;
            float dy = car.target_y - g_prev_ty_raw;
            if (sqrtf(dx * dx + dy * dy) > MERGE_JUMP_CM) {
                // 用跳变前(上一帧)的旧信标距离算倒计时, 不是新信标
                g_cd_end_ms = cnt + ComputeArrivalMs(g_prev_car_dist) + 500;
                g_cd_vx_cm = g_last_vx * ENCODER_MPS_TO_CMPS;
                g_cd_vy_cm = g_last_vy * ENCODER_MPS_TO_CMPS;
                g_state = TRACK_COUNTDOWN;
                break;
            }
        }
        g_prev_tx_raw = car.target_x;
        g_prev_ty_raw = car.target_y;
        g_prev_car_dist = car.car_target_dist;
        g_has_prev = 1;

        // --- 全速 0.6m/s ---
        {
            float rad = angle * ((float)M_PI / 180.0f);
            g_last_vx = TRACK_SPEED_MS * cosf(rad);
            g_last_vy = TRACK_SPEED_MS * sinf(rad);
            Mecanum_Set_Velocity(g_last_vx, g_last_vy, 0.0f);
        }

        g_is_edge = (fabsf(car.target_y_f) > EDGE_Y_THRESHOLD);
        if (g_is_edge) {
            if (g_edge_cnt < 65535) g_edge_cnt++;
        } else {
            g_edge_cnt = 0;
        }
        break;
    }

    case TRACK_COUNTDOWN:
    {
        // 倒计时滑行: 信任最后已知的信标位置, 保持原速度原方向直线前进
        // 期间不检查任何新目标 (防止被其他信标干扰或被短暂的假目标拉偏)
        // 速度保持恒定 (COUNTDOWN_SPEED_DECAY=1.0 → 不衰减)

        if (cnt >= g_cd_end_ms) {
            // 倒计时结束, 仍未重新捕获 → 标记丢失, 等待新目标验证
            g_state = TRACK_LOST;
            break;
        }

        g_cd_vx_cm *= COUNTDOWN_SPEED_DECAY;
        g_cd_vy_cm *= COUNTDOWN_SPEED_DECAY;
        Mecanum_Set_Velocity(g_cd_vx_cm / ENCODER_MPS_TO_CMPS,
                             g_cd_vy_cm / ENCODER_MPS_TO_CMPS, 0.0f);
        break;
    }

    case TRACK_LOST:
        Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        if (target_present) {
            // 所有新目标必须通过暂态验证 (绝不允许直接跟踪)
            memset(g_val_win, 0, sizeof(g_val_win));
            g_val_idx = 0;
            g_val_hits = 0;
            g_val_has_prev_pos = 0;
            g_val_lost = 0;
            g_state = TRACK_VALIDATING;
        }
        break;
    }
}
