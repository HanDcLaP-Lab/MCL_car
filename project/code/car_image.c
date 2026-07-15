#include "car_image.h"
#include <math.h>
#include <string.h>

// ================== 视觉跟踪全局变量 ==================
int rush_sign = 0;                         // Dash调试标志：触发/保持期间为1，不参与控制判定
float dist_out = 0;                        // 最近一次正常state3解算的车-信标距离 (cm，仅调试)
volatile float visual_last_vx = 0.0f;      // 当前保留的车体系X向速度指令 (m/s，前为正)
volatile float visual_last_vy = 0.0f;      // 当前保留的车体系Y向速度指令 (m/s，右为正)
volatile uint32_t dash_end_time = 0;        // Dash绝对结束时刻 (sys_time_ms，0表示未激活)
static volatile uint32_t post_dash_hold_end_time = 0; // Dash减速与静止等待的绝对结束时刻
volatile uint32_t rush_cooldown_end_time = 0; // 下次允许触发Dash的绝对时刻 (0表示无冷却)

// ================== 保质期槽位 (目标身份滤波) ==================
typedef struct {
    float x;
    float y;
    uint32_t expire_ms;
    uint8_t valid;
} Position_Slot_t;

typedef struct {
    float angle;           // 小车车体系方向 (rad)
    float distance;        // 当前候选对应的车-信标距离 (cm)
    uint32_t expire_ms;
    uint8_t valid;
} Direction_Slot_t;

static const uint8_t target_x_uart_index[TARGET_CANDIDATE_COUNT] = {2U, 8U, 10U};
static const uint8_t target_y_uart_index[TARGET_CANDIDATE_COUNT] = {3U, 9U, 11U};

static Position_Slot_t car_slot;                              // 最近可信小车坐标
static Position_Slot_t target_slot[TARGET_CANDIDATE_COUNT];  // 按无人机评分排序的信标坐标
static Direction_Slot_t latest_angle[TARGET_CANDIDATE_COUNT]; // 三个信标当前合成出的方向
static Direction_Slot_t adopted_angle;                        // 当前允许控制使用的方向
static Direction_Slot_t pending_angle;                        // adopted失效后的候补方向
static uint32_t adopted_angle_stable_ms = 0; // 同方向可信合成累计时间，用于扩展adopted保质期

static uint8_t position_slot_check(Position_Slot_t *slot) {
    if (slot->valid && sys_time_ms >= slot->expire_ms) slot->valid = 0;
    return slot->valid;
}

static uint8_t direction_slot_check(Direction_Slot_t *slot) {
    if (slot->valid && sys_time_ms >= slot->expire_ms) slot->valid = 0;
    return slot->valid;
}

static uint8_t angle_matches(float a, float b) {
    return cosf(a - b) >= ANGLE_MATCH_COS;
}

static void pending_angle_reset(void) {
    pending_angle.valid = 0;
}

static int8_t latest_match_index(float reference_angle) {
    for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
        if (direction_slot_check(&latest_angle[i]) &&
            angle_matches(latest_angle[i].angle, reference_angle)) {
            return (int8_t)i;
        }
    }
    return -1;
}

static int8_t latest_first_valid_index(void) {
    for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
        if (direction_slot_check(&latest_angle[i])) return (int8_t)i;
    }
    return -1;
}

// 新候选均与pending不一致时，用其中方向最接近旧pending的一项更新候补槽。
static int8_t latest_nearest_index(float reference_angle) {
    int8_t nearest = -1;
    float best_similarity = -2.0f;

    for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
        if (!direction_slot_check(&latest_angle[i])) continue;
        float similarity = cosf(latest_angle[i].angle - reference_angle);
        if (similarity > best_similarity) {
            best_similarity = similarity;
            nearest = (int8_t)i;
        }
    }
    return nearest;
}

// ================== 视觉状态机共享静态变量 ==================
// track_memory_ms 是“可靠双目标跟踪时长置信度”，不是继续运动的倒计时。
// state3 逐步累加、state1/2 按实际间隔衰减，达到阈值后才允许触发 Dash。
static uint32_t track_memory_ms = 0;       // 累积的可靠双目标跟踪置信时间 (ms，最大1000)
static uint32_t track_memory_last_ms = 0;  // 上次消费视觉数据时的sys_time_ms
static uint32_t track_memory_step_ms = 0;  // 本次与上次视觉处理的实际间隔 (ms)
static uint8_t  track_memory_started = 0;  // 1表示last_ms已经完成首次初始化
static uint8_t  zero_consecutive = 0;      // 连续消费到state0的次数 (最大计到2)
static float    prev_target_x = 0.0f, prev_target_y = 0.0f; // 保留的上一信标坐标，目前仅记录
static uint8_t  has_prev_target = 0;       // 上一信标坐标记录标志，目前不参与控制判断

static void adopted_angle_reset(void) {
    adopted_angle.valid = 0;
    adopted_angle_stable_ms = 0;
}

// 仅由本次重新合成的同方向调用。稳定时间按实际收包间隔累计，单帧最多计20ms；
// 保质期始终从本次可信方向合成时刻起算，未产生新方向时不会延后到期时刻。
static void adopted_angle_refresh(const Direction_Slot_t *candidate, uint32_t now) {
    uint32_t add_ms = track_memory_step_ms;
    if (add_ms > TRACK_STEP_MAX_MS) add_ms = TRACK_STEP_MAX_MS;
    if (adopted_angle_stable_ms >= ADOPTED_ANGLE_FULL_CONFIDENCE_MS ||
        add_ms > ADOPTED_ANGLE_FULL_CONFIDENCE_MS - adopted_angle_stable_ms) {
        adopted_angle_stable_ms = ADOPTED_ANGLE_FULL_CONFIDENCE_MS;
    } else {
        adopted_angle_stable_ms += add_ms;
    }

    uint32_t valid_ms = ANGLE_VALID_MS +
        (ADOPTED_ANGLE_MAX_VALID_MS - ANGLE_VALID_MS) * adopted_angle_stable_ms /
        ADOPTED_ANGLE_FULL_CONFIDENCE_MS;
    adopted_angle.angle = candidate->angle;
    adopted_angle.distance = candidate->distance;
    adopted_angle.expire_ms = now + valid_ms;
    adopted_angle.valid = 1;
}

// ================== Dash 估计 (EMA 滤波) ==================
static float dash_dir_vx_est = 0;       // Dash方向单位向量EMA的X分量
static float dash_dir_vy_est = 0;       // Dash方向单位向量EMA的Y分量
static float dash_speed_est = 0;        // 车向信标接近速度的EMA估计 (m/s)
static float dash_dist_prev = 0;        // 上次参与速度差分的车-信标距离 (cm)
static uint32_t dash_stamp_prev = 0;    // dash_dist_prev对应的sys_time_ms
static uint8_t dash_speed_samples = 0;  // 已接纳的正接近速度样本数
static float dash_dist_est = 0;         // Dash触发距离的EMA估计 (cm)

// 清除跨帧Dash估计；不改变Dash计时、冷却和跟踪置信度。
static void dash_estimate_reset(void) {
    dash_dir_vx_est = 0; dash_dir_vy_est = 0;
    dash_speed_est = 0; dash_dist_prev = 0; dash_stamp_prev = 0;
    dash_speed_samples = 0; dash_dist_est = 0;
}

static uint32_t Dash_Calculate_Duration_Ms(float distance_cm, float closing_speed) {
    float distance_m = distance_cm * 0.01f;
    // 总距离拆成“定速 Dash 距离 + 归零后的斜坡制动距离”。
    // 先扣除 v^2/(2a)，避免 Dash 结束时才减速造成系统性过冲。
    float stop_distance = closing_speed * closing_speed / (2.0f * MAX_ACCEL_LINEAR);
    float dash_distance = distance_m - stop_distance;

    if (dash_distance <= 0.0f) return 0U;

    uint32_t duration_ms = (uint32_t)(dash_distance / closing_speed * 1000.0f);
    if (duration_ms > DASH_MS_MAX) duration_ms = DASH_MS_MAX;
    return duration_ms + 100U;
}

// VISUAL_VELOCITY_GUARD_BEGIN
// 主循环负责发布视觉速度，1ms ISR 可能在其间令 Dash 到期并强制归零。
// Dash到期ISR会推进代次；当前视觉帧只能发布同一代次内的速度，
// 防止 ISR 刚停车，主循环又把处理中途的旧视觉指令写回 target_vel。
static volatile uint32_t visual_stop_epoch = 0U; // Dash到期ISR最近一次归零后的代次
static uint32_t visual_frame_stop_epoch = 0U;    // 当前Visual_Control_Loop入口快照的代次

static void Visual_Begin_Velocity_Frame(void) {
    visual_frame_stop_epoch = visual_stop_epoch;
}

static void Visual_Invalidate_Velocity(void) {
    visual_stop_epoch++;
    target_vel.vx = 0.0f;
    target_vel.vy = 0.0f;
    target_vel.wz = 0.0f;
}

static uint8_t Visual_Set_Velocity(float vx, float vy, float wz) {
    uint32_t expected_epoch = visual_frame_stop_epoch;

    if (visual_stop_epoch != expected_epoch) {
        return 0U;
    }

    Mecanum_Set_Velocity(vx, vy, wz);

    // ISR 若在三个速度分量写入期间停车，撤销可能残留的部分或完整旧指令。
    if (visual_stop_epoch != expected_epoch) {
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        return 0U;
    }

    return 1U;
}
// VISUAL_VELOCITY_GUARD_END

// ================== 静态函数前置声明 ==================
static void State0_Handler(void);
static void State12_Handler(uint8_t locked_state);
static void State3_Handler(void);
static uint8_t target_filter_update(uint8_t locked_state);
static void Visual_Track_Begin_Frame(void);
static void Visual_Track_Refresh(void);
static void Visual_Track_Decay(void);
static void Visual_Track_Clear(void);
static uint8_t Visual_Track_Is_Locked(void);

static void Visual_Track_Begin_Frame(void) {
    // 使用真实收包间隔更新置信度；单次间隔仍会在 Refresh 中限为20ms，
    // 避免主循环卡顿后用一个迟到的数据包一次性补满跟踪置信度。
    uint32_t now = sys_time_ms;
    if (!track_memory_started) {
        track_memory_started = 1;
        track_memory_last_ms = now;
        track_memory_step_ms = 0;
        return;
    }

    track_memory_step_ms = now - track_memory_last_ms;
    track_memory_last_ms = now;
    if (track_memory_step_ms > TRACK_MEMORY_MS) track_memory_step_ms = TRACK_MEMORY_MS;
}

static void Visual_Track_Refresh(void) {
    uint32_t add_ms = track_memory_step_ms;
    if (add_ms > TRACK_STEP_MAX_MS) add_ms = TRACK_STEP_MAX_MS;
    if (track_memory_ms + add_ms > TRACK_MEMORY_MS) track_memory_ms = TRACK_MEMORY_MS;
    else track_memory_ms += add_ms;
}

static void Visual_Track_Decay(void) {
    if (track_memory_ms > track_memory_step_ms) track_memory_ms -= track_memory_step_ms;
    else track_memory_ms = 0;
}

static void Visual_Track_Clear(void) {
    track_memory_ms = 0;
    track_memory_last_ms = sys_time_ms;
    track_memory_step_ms = 0;
    track_memory_started = 1;
}

static uint8_t Visual_Track_Is_Locked(void) {
    return track_memory_ms >= TRACK_LOCK_THRESHOLD_MS;
}

// ================== 现有函数 (不变) ==================

void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
    adopted_angle_reset();
    dash_estimate_reset();
}

void Image_Solve(float car_yaw, float *dist, float *angle) {
    // 直接获取无人机解算好的物理坐标 (单位: cm)
    float x_car = uart_data[0];       // [0] 小车地面X坐标
    float y_car = uart_data[1];       // [1] 小车地面Y坐标
    float x_target = uart_data[2];    // [2] 目标地面X坐标
    float y_target = uart_data[3];    // [3] 目标地面Y坐标
    float yaw_drone = uart_data[4];   // [4] 无人机已校正的地面系偏航角 (deg)

    // 1. 计算无人机坐标系下的相对矢量 (X:前, Y:右)
    float dx = x_target - x_car;
    float dy = y_target - y_car;

    // 2. 计算距离 (cm)
    *dist = sqrtf(dx * dx + dy * dy);

    // 3. 坐标系旋转: 无人机坐标系 -> 小车坐标系
    // 下传数据X前Y右，小车X前Y左，Yaw均为顺时针正
    // 旋转角 delta = Car_Yaw - Drone_Yaw
    float delta_rad = (car_yaw - yaw_drone) * ((float)(M_PI / 180.0));

    // 旋转矩阵 (顺时针旋转坐标系/逆时针旋转向量)
    // dx_car = dx * cos(delta) + dy * sin(delta)
    // dy_car = -dx * sin(delta) + dy * cos(delta)
    float dx_car = dx * cosf(delta_rad) + dy * sinf(delta_rad);
    float dy_car = -dx * sinf(delta_rad) + dy * cosf(delta_rad);

    dy_car = -dy_car; // Y轴取反适配小车坐标系

    // 4. 计算角度 (0度为车头, 90度为车右, 符合 atan2(y, x) 定义)
    *angle = atan2f(dy_car, dx_car) * (180.0f / (float)M_PI);
}

// ================== 目标身份滤波 ==================

static void position_slot_refresh(Position_Slot_t *slot, float x, float y,
                                  uint32_t now, uint32_t valid_ms) {
    slot->x = x;
    slot->y = y;
    slot->expire_ms = now + valid_ms;
    slot->valid = 1;
}

// 将本帧三个信标分别解算到小车车体系。第一候选沿用无人机滤波距离，
// 其余候选没有独立距离字段，因此使用同一对原始坐标计算距离。
static void latest_angles_update(uint32_t now) {
    float delta = (imu_car_rc_data.yaw - uart_data[4]) * ((float)M_PI / 180.0f);
    float cos_delta = cosf(delta);
    float sin_delta = sinf(delta);

    for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
        latest_angle[i].valid = 0;
        if (!position_slot_check(&car_slot) ||
            !position_slot_check(&target_slot[i])) continue;

        float dx = target_slot[i].x - car_slot.x;
        float dy = target_slot[i].y - car_slot.y;
        float dx_car = dx * cos_delta + dy * sin_delta;
        float dy_car = dx * sin_delta - dy * cos_delta;
        float raw_distance = sqrtf(dx * dx + dy * dy);

        latest_angle[i].angle = atan2f(dy_car, dx_car);
        latest_angle[i].distance =
            (i == 0U && uart_data[7] > 0.01f) ? uart_data[7] : raw_distance;
        latest_angle[i].expire_ms = now + ANGLE_VALID_MS;
        latest_angle[i].valid = 1;
    }
}

static void adopted_angle_replace(const Direction_Slot_t *candidate, uint8_t large_turn) {
    adopted_angle = *candidate;
    adopted_angle_stable_ms = 0;
    pending_angle_reset();
    if (large_turn) Mecanum_Set_Large_Turn_Accel_Limit(1U);
    dash_estimate_reset();
}

/**
 * @brief 更新短时坐标缓存，维护当前采纳方向和大角度候选方向
 * @param locked_state 当前视觉可见状态，仅决定本次刷新哪些坐标槽
 * @return 1表示存在可用adopted方向且visual_last_vx/vy已更新，0表示当前无可用方向
 *
 * state1/2可用当前坐标与50ms内的缓存坐标重新合成方向；只有本次实际合成出的
 * latest才能刷新adopted或维护pending，pending仅在adopted到期后接管。
 */
static uint8_t target_filter_update(uint8_t locked_state) {
    uint32_t now = sys_time_ms;

    // ① 按当前可见状态刷新小车和三个按评分排序的信标坐标槽。
    // 50ms槽位允许state1/2用当前坐标与短时缓存坐标合成可信方向。
    if (locked_state == 1 || locked_state == 3) {
        position_slot_refresh(&car_slot, uart_data[0], uart_data[1], now, CAR_VALID_MS);
    }
    if (locked_state == 2 || locked_state == 3) {
        for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
            position_slot_refresh(&target_slot[i],
                                  uart_data[target_x_uart_index[i]],
                                  uart_data[target_y_uart_index[i]],
                                  now, TARGET_VALID_MS);
        }
    }

    // ② 每次先失效旧latest；仅两侧坐标仍新鲜的候选在本次重新生成。
    latest_angles_update(now);

    // ③ fresh latest先维护候补槽，再由adopted决定刷新当前方向或在到期后接管候补。
    uint8_t adopted_ok = direction_slot_check(&adopted_angle);
    uint8_t pending_ok = direction_slot_check(&pending_angle);
    int8_t first_latest = latest_first_valid_index();

    if (first_latest >= 0) {
        int8_t replacement = pending_ok ? latest_match_index(pending_angle.angle) : -1;
        if (replacement < 0) {
            replacement = pending_ok ?
                latest_nearest_index(pending_angle.angle) : first_latest;
        }
        pending_angle = latest_angle[replacement];
        pending_ok = 1;
    }

    if (adopted_ok) {
        int8_t adopted_match = latest_match_index(adopted_angle.angle);
        if (adopted_match >= 0) {
            adopted_angle_refresh(&latest_angle[adopted_match], now);
            pending_angle_reset();
        }
    } else {
        adopted_angle_stable_ms = 0;
        if (pending_ok) {
            uint8_t is_large_reacquire =
                (dash_dir_vx_est != 0.0f || dash_dir_vy_est != 0.0f) &&
                !angle_matches(pending_angle.angle, adopted_angle.angle);
            adopted_angle_replace(&pending_angle, is_large_reacquire);
            adopted_ok = 1;
        }
    }

    // ④ 将身份滤波结果转换为固定模长速度方向，供State12/State3继续处理。
    if (adopted_ok) {
        visual_last_vx = TARGET_SPEED * cosf(adopted_angle.angle);
        visual_last_vy = TARGET_SPEED * sinf(adopted_angle.angle);
        return 1;
    }

    visual_last_vx = 0;
    visual_last_vy = 0;
    return 0;
}


// ================== 状态处理函数 ==================

// 状态 0：全丢 → 停车 + 清零
static void State0_Handler(void) {
    Mecanum_Set_Large_Turn_Accel_Limit(0U);
    Visual_Set_Velocity(0.0f, 0.0f, 0.0f);
    Visual_Track_Clear();
    has_prev_target = 0;
    dash_end_time = 0;
    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    car_slot.valid = 0;
    for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
        target_slot[i].valid = 0;
        latest_angle[i].valid = 0;
    }
    adopted_angle_reset();
    pending_angle_reset();
    dash_estimate_reset();
}

// 状态 1 或 2：单目标丢失
static void State12_Handler(uint8_t locked_state) {
    // Dash跟踪置信度仍随时间衰减；50ms内可用一新一旧坐标刷新方向置信度，
    // 此后若没有新合成方向，继续运动时间由adopted原有动态保质期决定。
    Visual_Track_Decay();
    if (!target_filter_update(locked_state)) {
        Visual_Set_Velocity(0.0f, 0.0f, 0.0f);
        return;
    }
    Visual_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
}

// 状态 3：双目标锁定 (正常追踪 + Dash 估计 + 近距离盲冲)
static void State3_Handler(void) {
    if (!target_filter_update(3)) {
        Visual_Set_Velocity(0.0f, 0.0f, 0.0f);
        return;
    }

    // 有 pending 候补：冻结旧方向，不让尚未接管的目标距离污染 Dash 估计。
    // 当前仍是 state3，因此继续累计“画面中确有车和信标”的跟踪置信度。
    if (pending_angle.valid) {
        Visual_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        Visual_Track_Refresh();
        return;
    }

    // ==================== Dash 方向 EMA (叠加在 adopted_angle 之上) ====================
    // 在单位圆的笛卡尔分量上做 EMA，再归一化，避免角度跨越 +/-180度时跳变。
    float unit_x = visual_last_vx / TARGET_SPEED;  // cos(adopted_angle)
    float unit_y = visual_last_vy / TARGET_SPEED;  // sin(adopted_angle)
    if (dash_dir_vx_est == 0 && dash_dir_vy_est == 0) {
        dash_dir_vx_est = unit_x;
        dash_dir_vy_est = unit_y;
    } else {
        dash_dir_vx_est = 0.3f * unit_x + 0.7f * dash_dir_vx_est;
        dash_dir_vy_est = 0.3f * unit_y + 0.7f * dash_dir_vy_est;
        float norm = sqrtf(dash_dir_vx_est*dash_dir_vx_est + dash_dir_vy_est*dash_dir_vy_est);
        if (norm > 0.001f) { dash_dir_vx_est /= norm; dash_dir_vy_est /= norm; }
    }

    // ==================== Dash 速度/距离 EMA + 可靠性门 ====================
    // 只接纳距离缩小时的正接近速度；至少4个有效样本后才使用估计值，
    // 样本不足时退回 TARGET_SPEED，距离本身也用 EMA 抑制末帧噪声。
    float raw_dist = adopted_angle.distance;
    if (dash_dist_prev > 0.01f && raw_dist > 0.01f && dash_stamp_prev > 0) {
        float dt = (float)(sys_time_ms - dash_stamp_prev) * 0.001f;
        if (dt > 0.0f) {
            float raw_speed = (dash_dist_prev - raw_dist) * 0.01f / dt;
            if (raw_speed > 0.0f) {
                if (dash_speed_est <= 0.0f) dash_speed_est = raw_speed;
                else dash_speed_est = 0.3f * raw_speed + 0.7f * dash_speed_est;
                if (dash_speed_samples < 255) dash_speed_samples++;
            }
        }
    }
    dash_dist_prev = raw_dist;
    dash_stamp_prev = sys_time_ms;
    if (dash_dist_est <= 0.0f) dash_dist_est = raw_dist;
    else dash_dist_est = 0.3f * raw_dist + 0.7f * dash_dist_est;

    // ==================== 盲冲触发 (距离低于阈值) ====================
    // 入口同时要求：距离有效且足够近、跟踪置信度达到150ms、当前不在冷却期。
    if (raw_dist > 0.01f && raw_dist < DASH_DIST_CM) {
        uint8_t is_cooldown = (rush_cooldown_end_time > 0 && sys_time_ms < rush_cooldown_end_time);
        if (dash_end_time == 0 && Visual_Track_Is_Locked() && !is_cooldown) {
            float dash_vx = dash_dir_vx_est * TARGET_SPEED;
            float dash_vy = dash_dir_vy_est * TARGET_SPEED;
            float dash_dist_cm = dash_dist_est > 0.0f ? dash_dist_est : raw_dist;

            float speed_sq = dash_vx * dash_vx + dash_vy * dash_vy;
            if (speed_sq >= 0.01f) {
                uint8_t speed_ok = (dash_speed_samples >= 4) && (dash_speed_est > DASH_SPEED_MIN_MPS);
                float closing = speed_ok ? dash_speed_est : TARGET_SPEED;
                if (closing < DASH_SPEED_MIN_MPS) closing = DASH_SPEED_MIN_MPS;
                else if (closing > DASH_SPEED_MAX_MPS) closing = DASH_SPEED_MAX_MPS;

                uint32_t duration_ms = Dash_Calculate_Duration_Ms(dash_dist_cm, closing);
                dash_end_time = sys_time_ms + duration_ms;
                Visual_Set_Velocity(dash_vx, dash_vy, 0.0f);
                rush_sign = 1;
                return; // 跳过正常追踪，下帧由 dash_end_time 门接管
            }
        }
    }

    // adopted已由本帧匹配到的具体候选刷新，方向和距离始终属于同一信标。
    dist_out = raw_dist;
    Visual_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);

    prev_target_x = uart_data[2];
    prev_target_y = uart_data[3];
    has_prev_target = 1;

    Visual_Track_Refresh();
}

// ================== 公开函数 ==================

// [重构] 抽离核心状态为全局，以便底层定时器与急停函数能强制干预。
// 本函数清除方向槽和 Dash 估计，但保留 track_memory_ms 与冷却时间；
// track_memory_ms 目前只在 State0_Handler() 中明确清零。
void Visual_State_Reset(void) {
    zero_consecutive = 0;
    dash_end_time = 0;
    Mecanum_Set_Large_Turn_Accel_Limit(0U);

    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    post_dash_hold_end_time = 0;
    car_slot.valid = 0;
    for (uint8_t i = 0; i < TARGET_CANDIDATE_COUNT; i++) {
        target_slot[i].valid = 0;
        latest_angle[i].valid = 0;
    }
    adopted_angle_reset();
    pending_angle_reset();
    dash_estimate_reset();
    // 注意：不在此函数内清零 rush_cooldown_end_time。
    // rush_cooldown 从预计刹停时刻开始保留 1 秒冷却期，
    // 其目的是防止 0 速度无限重入。若被一并清零，冷却形同虚设，
    // 信标闪烁时 state 3 可无限触发新一轮盲冲。
}

// ISR 级时间刹车检查：盲冲到期硬处理
// 由 Mecanum_Control_Loop (1ms ISR) 调用，作为主循环串口无数据时的最后防线
void Visual_Brake_Check(void) {
    if (dash_end_time > 0 && sys_time_ms >= dash_end_time) {
        // 先按当前平滑速度估算归零所需时间，再清理 Dash 状态。
        // post_dash_hold 覆盖“斜坡减速 + 完全停稳后的短等待”，冷却从预计停稳时刻起算。
        float braking_speed = sqrtf(smooth_vx * smooth_vx + smooth_vy * smooth_vy);
        uint32_t braking_time_ms =
            (uint32_t)(braking_speed / MAX_ACCEL_LINEAR * 1000.0f + 0.999f);
        Visual_Invalidate_Velocity();
        Visual_State_Reset();
        post_dash_hold_end_time = sys_time_ms + braking_time_ms + POST_DASH_HOLD_MS;
        rush_cooldown_end_time = sys_time_ms + braking_time_ms + 1000U;
    }
}

void Visual_Control_Loop(void) {
    // 处理优先级：底盘锁定 > 连续state0 > 正在Dash > Dash后制动/等待 > 常规状态。
    // 因此确定熄灯的 state0 可以打断 Dash，其余视觉变化不能改变已启动的 Dash 时长。
    Visual_Begin_Velocity_Frame();
#if 0
    static uint32_t last_time = 0;
    uint32_t current_time = sys_time_ms;
    uint32_t dt = current_time - last_time;
    last_time = current_time;

    // 调试真实执行间隔时可临时打开，常开会干扰控制周期和无线带宽。
    wireless_uart_send_string("dt:");
    wireless_uart_send_int((int32_t)dt);
    wireless_uart_send_string("\r\n");
#endif

    if (!Chassis_Is_Armed()) return;
    Visual_Track_Begin_Frame();
    uint8_t locked_state = (uint8_t)uart_data[5];

    // 连续两次控制调用收到 state0：信标确定熄灭，强制中断一切滑行/盲冲。
    // 这里统计的是主循环实际消费次数，不是 UART 解析器内部见到的物理包数量。
    if (locked_state == 0) {
        if (zero_consecutive < 2U) {
            zero_consecutive++;
        }
        if (zero_consecutive < 2U) {
            return; // 首个 state0 仅确认，不让后续 switch 提前执行 State0_Handler
        }

        State0_Handler();
        rush_sign = 0;
        return;
    } else {
        zero_consecutive = 0;
    }

    // 盲冲计时器激活期间：无视后续非零视觉状态，重复发布 visual_last 直到到期。
    // 彻底解决无人机丢包、相机曝光导致单帧时长被放大所引发的冲刺距离失控问题。
    if (dash_end_time > 0) {
        Visual_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        rush_sign = 1;
        return; // 提前退出，屏蔽后续视觉解析！
    }

    // Dash结束后先完成斜坡减速，再静止等待画面稳定并采纳新目标。
    if (post_dash_hold_end_time > 0) {
        if (sys_time_ms < post_dash_hold_end_time) {
            Visual_Set_Velocity(0.0f, 0.0f, 0.0f);
            return;
        }
        post_dash_hold_end_time = 0;
    }

    // 0: 全丢, 1: 仅小车, 2: 仅信标, 3: 都有
    rush_sign = 0;

    if (locked_state == 0) {
        has_prev_target = 0;
    }

    switch (locked_state) {
        case 0:  State0_Handler();  break;
        case 1:  /* fall through */
        case 2:  State12_Handler(locked_state); break;
        case 3:  State3_Handler();  break;
        default: break;
    }
}
