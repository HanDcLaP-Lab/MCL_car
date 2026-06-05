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



CarCtrl_Context_t ctx;

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
    memset(&ctx, 0, sizeof(CarCtrl_Context_t));
    ctx.state = TRACK_IDLE;
    g_predicted_arrival_ms = 0;
}

CarCtrl_State_t CarCtrl_GetState(void) {
    return ctx.state;
}

void CarCtrl_Update(void) {
    if (!target_vel.unlock) {
        CarCtrl_Init();
        return;
    }

    uint8_t locked_state = (uint8_t)(car.locked_state + 0.5f);
    // 无人机协议中 bit1(值为2)表示信标存在，bit0(值为1)表示小车存在
    uint8_t target_present = ((locked_state & 0x02) != 0); 

    switch (ctx.state) {

    case TRACK_IDLE:
        if (target_present) {
            // 所有新目标必须通过暂态验证 (绝不允许直接跟踪)
            memset(ctx.val_win, 0, sizeof(ctx.val_win));
            ctx.val_idx = 0;
            ctx.val_hits = 0;
            ctx.val_has_prev_pos = 0;
            ctx.val_lost = 0;
            ctx.state = TRACK_VALIDATING;
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
            ctx.val_lost++;
#if VALIDATE_LOST_TIMEOUT_FRAMES > 0
            if (ctx.val_lost > VALIDATE_LOST_TIMEOUT_FRAMES) {
                ctx.state = TRACK_IDLE;
                Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
                ctx.val_lost = 0;
                ctx.val_has_prev_pos = 0;
                break;
            }
#endif
        } else {
            ctx.val_lost = 0;
        }

        // --- 条件2: 坐标连续性检查 (同一信标判定) ---
        uint8_t position_ok = 1;
        float tx_earth = 0.0f, ty_earth = 0.0f;
        if (target_present) {
            // 【关键修复】取消减去 car_x/y！小车一旦丢帧 car_x 就会冻结，此时无人机的自旋会导致相对坐标在绝对系中剧烈甩动(>50cm)，从而无限重置验证！
            // 直接使用 target_x/y (目标相对无人机)，由于相邻两帧无人机的物理位移极小，绝对不会触发 50cm 的跳变误判。
            float yaw_rad = car.drone_yaw * ((float)M_PI / 180.0f);
            float cos_yaw = cosf(yaw_rad);
            float sin_yaw = sinf(yaw_rad);
            tx_earth = car.target_x * cos_yaw - car.target_y * sin_yaw;
            ty_earth = car.target_x * sin_yaw + car.target_y * cos_yaw;

            if (ctx.val_has_prev_pos) {
                float dx = tx_earth - ctx.val_last_tx;
                float dy = ty_earth - ctx.val_last_ty;
                float jump = sqrtf(dx * dx + dy * dy);
                ctx.last_jump = jump; // 保存到上下文供串口打印
                position_ok = (jump <= VALIDATE_MAX_JUMP_CM);
            }
            // 首次有效帧: 无条件通过, 记录坐标作为后续比较基准
            if (position_ok) {
                ctx.val_last_tx = tx_earth;
                ctx.val_last_ty = ty_earth;
                ctx.val_has_prev_pos = 1;
            }
        }

        // 坐标不连续 → 判定为不同信标 → 重置验证
        if (!position_ok) {
            memset(ctx.val_win, 0, sizeof(ctx.val_win));
            ctx.val_idx = 0;
            ctx.val_hits = 0;
            ctx.val_has_prev_pos = 0;
        }

        // --- 条件1: 滑动窗口更新 (仅在坐标连续时写入, 防止不同信标的帧污染窗口) ---
        if (position_ok) {
            uint8_t old = ctx.val_win[ctx.val_idx];
            ctx.val_win[ctx.val_idx] = target_present ? 1 : 0;
            if (target_present && !old) ctx.val_hits++;
            if (!target_present && old) ctx.val_hits--;
            ctx.val_idx = (ctx.val_idx + 1) % VALIDATE_WINDOW_FRAMES;
        }

        // --- 双条件同时满足 → 进入 ACTIVE 追踪 ---
        if (ctx.val_hits >= VALIDATE_MIN_HITS) {
            ctx.state = TRACK_ACTIVE;
            ctx.active_frames = 0;
            ctx.edge_cnt = 0;
            ctx.is_edge = 0;
            ctx.has_prev = 0;
        }

        // --- 运动控制: 全速 0.6m/s, 缺帧也不减速 ---
        if (target_present) {
            float dist = 0.0f, angle = 0.0f;
            Image_Solve(imu_car_rc_data.yaw, &dist, &angle);
            float rad = angle * ((float)M_PI / 180.0f);
            ctx.last_vx = TRACK_SPEED_MS * cosf(rad);
            ctx.last_vy = TRACK_SPEED_MS * sinf(rad);
            Mecanum_Set_Velocity(ctx.last_vx, ctx.last_vy, 0.0f);
        } else {
            Mecanum_Set_Velocity(ctx.last_vx, ctx.last_vy, 0.0f);
        }
        break;
    }

    case TRACK_ACTIVE:
    {
        ctx.active_frames++;

        // --- 目标丢失 → 用上一帧(目标还在时)算好的预测时间 ---
        if (!target_present) {
            if (ctx.active_frames >= MIN_ACTIVE_BEFORE_CD_FRAMES) {
                ctx.cd_end_ms = cnt + g_predicted_arrival_ms;
                ctx.cd_vx_cm = ctx.last_vx * ENCODER_MPS_TO_CMPS;
                ctx.cd_vy_cm = ctx.last_vy * ENCODER_MPS_TO_CMPS;
                ctx.state = TRACK_COUNTDOWN;
            } else {
                ctx.state = TRACK_LOST;
            }
            break;
        }

        // --- 目标存在: 解算距离角度, 更新预测, 跳变检测 ---
        float dist = 0.0f, angle = 0.0f;
        Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

        g_predicted_arrival_ms = ComputeArrivalMs(dist);

        // 同理，取消减去 car_x/y，消除连带丢失造成的假跳变
        float yaw_rad = car.drone_yaw * ((float)M_PI / 180.0f);
        float cos_yaw = cosf(yaw_rad);
        float sin_yaw = sinf(yaw_rad);
        float tx_earth = car.target_x * cos_yaw - car.target_y * sin_yaw;
        float ty_earth = car.target_x * sin_yaw + car.target_y * cos_yaw;

        if (ctx.has_prev && ctx.prev_car_dist < MERGE_JUMP_MAX_DIST_CM) {
            float dx = tx_earth - ctx.prev_tx_raw;
            float dy = ty_earth - ctx.prev_ty_raw;
            float jump = sqrtf(dx * dx + dy * dy);
            ctx.last_jump = jump; // 保存 ACTIVE 状态下的跳变距离供串口打印
            if (jump > MERGE_JUMP_CM) {
                // 用跳变前(上一帧)的旧信标距离算倒计时, 不是新信标
                ctx.cd_end_ms = cnt + ComputeArrivalMs(ctx.prev_car_dist) + 1000;
                ctx.cd_vx_cm = ctx.last_vx * ENCODER_MPS_TO_CMPS;
                ctx.cd_vy_cm = ctx.last_vy * ENCODER_MPS_TO_CMPS;
                ctx.state = TRACK_COUNTDOWN;
                break;
            }
        }
        ctx.prev_tx_raw = tx_earth;
        ctx.prev_ty_raw = ty_earth;
        ctx.prev_car_dist = dist; // 弃用被无人机废弃的 car_target_dist, 改用实时的 dist
        ctx.has_prev = 1;

        // --- 全速 0.6m/s ---
        {
            float rad = angle * ((float)M_PI / 180.0f);
            ctx.last_vx = TRACK_SPEED_MS * cosf(rad);
            ctx.last_vy = TRACK_SPEED_MS * sinf(rad);
            Mecanum_Set_Velocity(ctx.last_vx, ctx.last_vy, 0.0f);
        }

        ctx.is_edge = (fabsf(car.target_y_f) > EDGE_Y_THRESHOLD);
        if (ctx.is_edge) {
            if (ctx.edge_cnt < 65535) ctx.edge_cnt++;
        } else {
            ctx.edge_cnt = 0;
        }
        break;
    }

    case TRACK_COUNTDOWN:
    {
        // 倒计时滑行: 信任最后已知的信标位置, 保持原速度原方向直线前进
        // 期间不检查任何新目标 (防止被其他信标干扰或被短暂的假目标拉偏)
        // 速度保持恒定 (COUNTDOWN_SPEED_DECAY=1.0 → 不衰减)

        if (cnt >= ctx.cd_end_ms) {
            // 倒计时结束, 仍未重新捕获 → 标记丢失, 等待新目标验证
            ctx.state = TRACK_LOST;
            break;
        }

        ctx.cd_vx_cm *= COUNTDOWN_SPEED_DECAY;
        ctx.cd_vy_cm *= COUNTDOWN_SPEED_DECAY;
        Mecanum_Set_Velocity(ctx.cd_vx_cm / ENCODER_MPS_TO_CMPS,
                             ctx.cd_vy_cm / ENCODER_MPS_TO_CMPS, 0.0f);
        break;
    }

    case TRACK_LOST:
        Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        if (target_present) {
            // 所有新目标必须通过暂态验证 (绝不允许直接跟踪)
            memset(ctx.val_win, 0, sizeof(ctx.val_win));
            ctx.val_idx = 0;
            ctx.val_hits = 0;
            ctx.val_has_prev_pos = 0;
            ctx.val_lost = 0;
            ctx.state = TRACK_VALIDATING;
        }
        break;
    }
}
