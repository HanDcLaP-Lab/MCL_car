#include "car_image.h"
#include <math.h>
#include <string.h>

// ================== 视觉跟踪全局变量 ==================
int rush_sign = 0;
float dist_out = 0;
volatile float visual_last_vx = 0.0f;
volatile float visual_last_vy = 0.0f;
volatile uint32_t dash_end_time = 0;          // 盲冲绝对结束物理时间
volatile uint32_t rush_cooldown_end_time = 0; // 防重入冷却绝对结束时间

// ================== 保质期槽位 (目标身份滤波) ==================
typedef struct {
    float     value;
    float     value_y;
    uint32_t  expire_ms;
    uint8_t   valid;
} Expiring_Slot_t;

static Expiring_Slot_t car_slot;        // 小车地面坐标
static Expiring_Slot_t target_slot;     // 目标地面坐标
static Expiring_Slot_t latest_angle;    // 最新合成角度
static Expiring_Slot_t adopted_angle;   // 当前采纳角度
static Expiring_Slot_t pending_angle;   // 被拒绝候选

static uint8_t slot_check(Expiring_Slot_t *s) {
    if (s->valid && sys_time_ms >= s->expire_ms) s->valid = 0;
    return s->valid;
}

// ================== 视觉状态机共享静态变量 ==================
static uint32_t track_memory_ms = 0;
static uint32_t track_memory_last_ms = 0;
static uint32_t track_memory_step_ms = 0;
static uint8_t  track_memory_started = 0;
static uint8_t  zero_consecutive = 0;
static float    prev_target_x = 0.0f, prev_target_y = 0.0f;
static uint8_t  has_prev_target = 0;

// ================== Dash 估计 (EMA 滤波) ==================
static float dash_dir_vx_est = 0;  // 方向向量 EMA (单位向量, X分量)
static float dash_dir_vy_est = 0;  // 方向向量 EMA (单位向量, Y分量)
static float dash_speed_est = 0;   // 接近速度 EMA (m/s)
static float dash_dist_prev = 0;   // 上帧距离 (cm)
static uint32_t dash_stamp_prev = 0;      // 上帧时间 (ms)
static uint8_t dash_speed_samples = 0;     // 可靠性门: 有效样本数
static float dash_dist_est = 0;   // 距离 EMA (cm)

// VISUAL_VELOCITY_GUARD_BEGIN
// ISR 每次强制停车都会推进代次；当前视觉帧只能发布同一代次内的速度。
static volatile uint32_t visual_stop_epoch = 0U;
static uint32_t visual_frame_stop_epoch = 0U;

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
    dash_dir_vx_est = 0; dash_dir_vy_est = 0;
    dash_speed_est = 0; dash_dist_prev = 0; dash_stamp_prev = 0;
    dash_speed_samples = 0; dash_dist_est = 0;
}

void Image_Solve(float car_yaw, float *dist, float *angle) {
    extern float uart_data[8];
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

static uint8_t target_filter_update(uint8_t locked_state) {
    uint32_t now = sys_time_ms;

    if (locked_state == 1 || locked_state == 3) {
        car_slot.value   = uart_data[0];
        car_slot.value_y = uart_data[1];
        car_slot.expire_ms = now + CAR_VALID_MS;
        car_slot.valid = 1;
    }
    if (locked_state == 2 || locked_state == 3) {
        target_slot.value   = uart_data[2];
        target_slot.value_y = uart_data[3];
        target_slot.expire_ms = now + TARGET_VALID_MS;
        target_slot.valid = 1;
    }

    if (slot_check(&car_slot) && slot_check(&target_slot)) {
        float dx = target_slot.value   - car_slot.value;
        float dy = target_slot.value_y - car_slot.value_y;
        float delta = (imu_car_rc_data.yaw - uart_data[4]) * ((float)M_PI / 180.0f);
        float dx_car = dx * cosf(delta) + dy * sinf(delta);
        float dy_car = -dx * sinf(delta) + dy * cosf(delta);
        dy_car = -dy_car;
        latest_angle.value = atan2f(dy_car, dx_car);
        latest_angle.expire_ms = now + ANGLE_VALID_MS;
        latest_angle.valid = 1;
    }

    uint8_t adopted_ok = slot_check(&adopted_angle);
    uint8_t latest_ok  = slot_check(&latest_angle);

    if (latest_ok) {
        if (!adopted_ok) {
            adopted_angle = latest_angle;
            pending_angle.valid = 0;
        } else {
            float cd = cosf(latest_angle.value) * cosf(adopted_angle.value)
                     + sinf(latest_angle.value) * sinf(adopted_angle.value);
            if (cd >= ANGLE_MATCH_COS) {
                adopted_angle = latest_angle;
                pending_angle.valid = 0;
            } else {
                pending_angle = latest_angle;
            }
        }
    }

    if (adopted_ok) {
        visual_last_vx = TARGET_SPEED * cosf(adopted_angle.value);
        visual_last_vy = TARGET_SPEED * sinf(adopted_angle.value);
        return 1;
    }

    if (slot_check(&pending_angle)) {
        adopted_angle = pending_angle;
        pending_angle.valid = 0;
        visual_last_vx = TARGET_SPEED * cosf(adopted_angle.value);
        visual_last_vy = TARGET_SPEED * sinf(adopted_angle.value);
        return 1;
    }

    visual_last_vx = 0;
    visual_last_vy = 0;
    return 0;
}


// ================== 状态处理函数 ==================

// 状态 0：全丢 → 停车 + 清零
static void State0_Handler(void) {
    Visual_Set_Velocity(0.0f, 0.0f, 0.0f);
    Visual_Track_Clear();
    has_prev_target = 0;
    dash_end_time = 0;
    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    car_slot.valid = 0;
    target_slot.valid = 0;
    latest_angle.valid = 0;
    adopted_angle.valid = 0;
    pending_angle.valid = 0;
    dash_dir_vx_est = 0; dash_dir_vy_est = 0;
    dash_speed_est = 0; dash_dist_prev = 0; dash_stamp_prev = 0;
    dash_speed_samples = 0; dash_dist_est = 0;
}

// 状态 1 或 2：单目标丢失
static void State12_Handler(uint8_t locked_state) {
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

    // ==================== Dash 方向 EMA (叠加在 adopted_angle 之上) ====================
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
    float raw_dist = uart_data[7];
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
    if (uart_data[7] > 0.01f && uart_data[7] < DASH_DIST_CM) {
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

                float duration_sec = (dash_dist_cm / 100.0f) / closing;
                int32_t duration_ms = (int32_t)(duration_sec * 1000.0f) + DASH_EXTRA_MS;
                if (duration_ms > (int32_t)DASH_MS_MAX) duration_ms = (int32_t)DASH_MS_MAX;
                if (duration_ms < (int32_t)DASH_MS_MIN) duration_ms = (int32_t)DASH_MS_MIN;
                dash_end_time = sys_time_ms + (uint32_t)duration_ms;
                Visual_Set_Velocity(dash_vx, dash_vy, 0.0f);
                rush_sign = 1;
                return; // 跳过正常追踪，下帧由 dash_end_time 门接管
            }
        }
    }

    // 有 pending 候选：adopted 未采纳新观测，方向保持冻结
    if (pending_angle.valid) {
        Visual_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        Visual_Track_Refresh();
        return;
    }

    // 正常追踪：adopted 已刷新 → 发布新速度
    float dist, angle;
    Image_Solve(imu_car_rc_data.yaw, &dist, &angle);
    dist_out = dist;

    float arad = angle * ((float)M_PI / 180.0f);
    float vx = TARGET_SPEED * cosf(arad);
    float vy = TARGET_SPEED * sinf(arad);
    Visual_Set_Velocity(vx, vy, 0.0f);
    visual_last_vx = vx;
    visual_last_vy = vy;

    prev_target_x = uart_data[2];
    prev_target_y = uart_data[3];
    has_prev_target = 1;

    Visual_Track_Refresh();
}

// ================== 公开函数 ==================

// [重构] 抽离核心状态为全局，以便底层定时器与急停函数能强制干预
void Visual_State_Reset(void) {
    zero_consecutive = 0;
    dash_end_time = 0;

    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    car_slot.valid = 0;
    target_slot.valid = 0;
    latest_angle.valid = 0;
    adopted_angle.valid = 0;
    pending_angle.valid = 0;
    dash_dir_vx_est = 0; dash_dir_vy_est = 0;
    dash_speed_est = 0; dash_dist_prev = 0; dash_stamp_prev = 0;
    dash_speed_samples = 0; dash_dist_est = 0;
    // 注意：不在此函数内清零 rush_cooldown_end_time。
    // rush_cooldown 是 dash 到期时由 1ms ISR 设置的 1 秒冷却期，
    // 其目的是防止 0 速度无限重入。若被一并清零，冷却形同虚设，
    // 信标闪烁时 state 3 可无限触发新一轮盲冲。
}

// ISR 级时间刹车检查：盲冲到期硬处理
// 由 Mecanum_Control_Loop (1ms ISR) 调用，作为主循环串口无数据时的最后防线
void Visual_Brake_Check(void) {
    if (dash_end_time > 0 && sys_time_ms >= dash_end_time) {
        Visual_Invalidate_Velocity();
        Visual_State_Reset();
        rush_cooldown_end_time = sys_time_ms + 1000;
    }
}

void Visual_Control_Loop(void) {
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

    // 连续两帧全丢（state 0）：信标确定熄灭，强制中断一切滑行/盲冲
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

    // 盲冲计时器激活期间：无视后续视觉状态，保持盲冲速度直到物理时间到期
    // 彻底解决无人机丢包、相机曝光导致单帧时长被放大所引发的冲刺距离失控问题。
    if (dash_end_time > 0) {
        Visual_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        rush_sign = 1;
        return; // 提前退出，屏蔽后续视觉解析！
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
