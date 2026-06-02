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

// 暂态验证窗口
static uint8_t g_val_win[VALIDATE_WINDOW_FRAMES];
static uint8_t g_val_idx;
static uint8_t g_val_hits;

// 倒计时
static int32_t g_cd_end_ms;
static float   g_cd_vx_cm, g_cd_vy_cm;

// 合并滑行
static uint16_t g_merge_frames;
static float g_last_vx, g_last_vy;

// 边缘
static uint16_t g_edge_cnt;
static uint8_t  g_is_edge;

// 跳变检测 (用原始 target_x/y)
static float g_prev_tx_raw, g_prev_ty_raw;
static float g_prev_car_dist;
static uint8_t g_has_prev;

// 调试：全流程预测到达时间
int32_t g_predicted_arrival_ms = 0;

// ===================== 内部函数 =====================

static int32_t ComputeCountdownMs(void) {
    float avg_spd_cm = (fabsf(encoder_data.lf) + fabsf(encoder_data.rf) +
                        fabsf(encoder_data.lb) + fabsf(encoder_data.rb))
                       * 0.25f * ENCODER_MPS_TO_CMPS;
    float dist_cm = car.car_target_dist;
    int32_t ms = COOLDOWN_MAX_MS;
    if (avg_spd_cm > 1.0f) {
        int32_t est = (int32_t)(dist_cm / avg_spd_cm * 1000.0f);
        if (est < ms) ms = est;
    }
    if (ms < COUNTDOWN_MIN_MS) ms = COUNTDOWN_MIN_MS;
    return ms;
}

// ===================== 接口函数 =====================

void CarCtrl_Init(void) {
    g_state = TRACK_IDLE;
    g_val_idx = 0;
    g_val_hits = 0;
    memset(g_val_win, 0, sizeof(g_val_win));
    g_cd_end_ms = 0;
    g_cd_vx_cm = 0.0f;
    g_cd_vy_cm = 0.0f;
    g_merge_frames = 0;
    g_last_vx = 0.0f;
    g_last_vy = 0.0f;
    g_edge_cnt = 0;
    g_is_edge = 0;
    g_prev_tx_raw = 0.0f;
    g_prev_ty_raw = 0.0f;
    g_prev_car_dist = 0.0f;
    g_has_prev = 0;
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
#if ENABLE_VALIDATION
            memset(g_val_win, 0, sizeof(g_val_win));
            g_val_idx = 0;
            g_val_hits = 0;
            g_state = TRACK_VALIDATING;
#else
            g_state = TRACK_ACTIVE;
#endif
        } else {
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        }
        break;

    case TRACK_VALIDATING:
    {
        // 滑动窗口更新
        uint8_t old = g_val_win[g_val_idx];
        g_val_win[g_val_idx] = target_present ? 1 : 0;
        if (target_present && !old) g_val_hits++;
        if (!target_present && old) g_val_hits--;
        g_val_idx = (g_val_idx + 1) % VALIDATE_WINDOW_FRAMES;

        if (g_val_hits >= VALIDATE_MIN_HITS) {
            g_state = TRACK_ACTIVE;
            g_edge_cnt = 0;
            g_is_edge = 0;
            g_has_prev = 0;
        }

        if (target_present) {
            float dist = 0.0f, angle = 0.0f;
            Image_Solve(imu_car_rc_data.yaw, &dist, &angle);
            float speed = (dist < TRACK_CLOSE_DIST_CM) ? TRACK_CLOSE_SPEED_MS : TRACK_SPEED_MS;
            float rad = angle * ((float)M_PI / 180.0f);
            g_last_vx = speed * cosf(rad);
            g_last_vy = speed * sinf(rad);
            Mecanum_Set_Velocity(g_last_vx, g_last_vy, 0.0f);
        } else {
            Mecanum_Set_Velocity(g_last_vx * 0.5f, g_last_vy * 0.5f, 0.0f);
        }
        g_merge_frames = 0;
        break;
    }

    case TRACK_ACTIVE:
    {
        g_predicted_arrival_ms = ComputeCountdownMs();

        if (!target_present) {
            g_cd_end_ms = cnt + g_predicted_arrival_ms;
            if (g_is_edge && g_edge_cnt < 5) {
                g_cd_end_ms = cnt + COUNTDOWN_MIN_MS;
            }
            g_cd_vx_cm = g_last_vx * ENCODER_MPS_TO_CMPS;
            g_cd_vy_cm = g_last_vy * ENCODER_MPS_TO_CMPS;
            g_state = TRACK_COUNTDOWN;
            break;
        }

        float dist = 0.0f, angle = 0.0f;
        Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

        // 合并跳变检测
        if (g_has_prev && g_prev_car_dist < MERGE_DIST_THRESHOLD) {
            float dx = car.target_x - g_prev_tx_raw;
            float dy = car.target_y - g_prev_ty_raw;
            if (sqrtf(dx * dx + dy * dy) > MERGE_JUMP_THRESHOLD) {
                g_merge_frames = MERGE_COAST_FRAMES;
            }
        }
        g_prev_tx_raw = car.target_x;
        g_prev_ty_raw = car.target_y;
        g_prev_car_dist = car.car_target_dist;
        g_has_prev = 1;

        if (g_merge_frames > 0) {
            g_merge_frames--;
            Mecanum_Set_Velocity(g_last_vx, g_last_vy, 0.0f);
        } else {
            float speed = (dist < TRACK_CLOSE_DIST_CM) ? TRACK_CLOSE_SPEED_MS : TRACK_SPEED_MS;
            float rad = angle * ((float)M_PI / 180.0f);
            g_last_vx = speed * cosf(rad);
            g_last_vy = speed * sinf(rad);
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
#if ENABLE_SMOOTH_SWITCH
        if (target_present) {
            float dx = car.target_x - g_prev_tx_raw;
            float dy = car.target_y - g_prev_ty_raw;
            if (sqrtf(dx * dx + dy * dy) < SWITCH_PROXIMITY_THRESHOLD) {
                g_state = TRACK_ACTIVE;
                g_edge_cnt = 0;
                g_is_edge = 0;
                break;
            }
        }
#else
        if (target_present) {
            g_state = TRACK_ACTIVE;
            g_edge_cnt = 0;
            g_is_edge = 0;
            break;
        }
#endif

        if (cnt >= g_cd_end_ms) {
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
#if ENABLE_VALIDATION
            memset(g_val_win, 0, sizeof(g_val_win));
            g_val_idx = 0;
            g_val_hits = 0;
            g_state = TRACK_VALIDATING;
#else
            g_state = TRACK_ACTIVE;
#endif
        }
        break;
    }
}
