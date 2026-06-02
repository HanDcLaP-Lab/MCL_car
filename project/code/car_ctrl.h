#ifndef _CAR_CTRL_H
#define _CAR_CTRL_H

#include "zf_common_headfile.h"

// ===================== 追踪状态机 =====================
typedef enum {
    TRACK_IDLE       = 0,
    TRACK_VALIDATING = 1,
    TRACK_ACTIVE     = 2,
    TRACK_COUNTDOWN  = 3,
    TRACK_LOST       = 4
} CarCtrl_State_t;

// ===================== 功能开关 =====================
#define ENABLE_COUNTDOWN        1
#define ENABLE_VALIDATION       1
#define ENABLE_SMOOTH_SWITCH    1

// ===================== 倒计时 (cm, cm/s) =====================
#define COOLDOWN_MAX_MS         3000
#define COUNTDOWN_MIN_MS        20
#define COUNTDOWN_SPEED_DECAY   0.98f
#define ENCODER_MPS_TO_CMPS     100.0f

// ===================== 暂态验证 =====================
#define VALIDATE_WINDOW_FRAMES  50
#define VALIDATE_MIN_HITS       30

// ===================== 平滑切换 =====================
#define SWITCH_PROXIMITY_THRESHOLD  50.0f

// ===================== 追踪速度 =====================
#define TRACK_SPEED_MS          0.6f
#define TRACK_CLOSE_DIST_CM     20.0f
#define TRACK_CLOSE_SPEED_MS    0.35f

// ===================== 合并检测 =====================
#define MERGE_DIST_THRESHOLD    28.0f
#define MERGE_JUMP_THRESHOLD    40.0f
#define MERGE_COAST_FRAMES      100

// ===================== 边缘检测 =====================
#define EDGE_Y_THRESHOLD        300.0f

// ===================== 接口 =====================
void CarCtrl_Init(void);
void CarCtrl_Update(void);
CarCtrl_State_t CarCtrl_GetState(void);

extern int32_t g_predicted_arrival_ms;

#endif
