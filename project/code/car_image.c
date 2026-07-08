#include "car_image.h"
#include <math.h>
#include <string.h>

// ================== 视觉跟踪全局变量 ==================
int rush_sign = 0;
float dist_out = 0;
volatile float visual_last_vx = 0.0f;
volatile float visual_last_vy = 0.0f;
volatile uint32_t visual_coast_end_time = 0;  // 目标丢失软滑行绝对物理计时器
volatile uint32_t merge_coast_end_time = 0;   // 信标跳变滑行物理计时器
volatile uint8_t  merge_coast_expired = 0;    // merge_coast ISR 到期标志，防竞态清零后跳变检测死循环
volatile uint8_t  visual_coast_expired = 0;   // visual_coast ISR 到期标志，防竞态清零后软滑行无限循环
volatile uint32_t dash_end_time = 0;          // [重构] 融合盲冲绝对结束物理时间
volatile uint32_t rush_cooldown_end_time = 0; // [重构] 防重入冷却绝对结束时间
volatile uint8_t  dash_source = 0;            // dash 触发来源: 1=状态4融合盲冲, 2=历史保留(状态1/2入口已关闭)

// ================== 视觉状态机共享静态变量 ==================
static uint16_t is_edge = 0;
static uint32_t track_memory_ms = 0;
static uint32_t track_memory_last_ms = 0;
static uint32_t track_memory_step_ms = 0;
static uint8_t  track_memory_started = 0;
static float    prev_target_x = 0.0f, prev_target_y = 0.0f;
static uint8_t  has_prev_target = 0;
static float    prev_car_dist = 0.0f;

// ================== 静态函数前置声明 ==================
static void State0_Handler(void);
static void State12_Handler(uint8_t locked_state);
static void State3_Handler(void);
static void State4_Handler(void);
static float Compute_Jump_Threshold(float car_dist);
static void Visual_Track_Begin_Frame(void);
static void Visual_Track_Refresh(void);
static void Visual_Track_Decay(void);
static void Visual_Track_Clear(void);
static void Visual_Clear_Soft_Coast(void);
static void Visual_Clear_Merge_Coast(void);
static uint8_t Visual_Track_Is_Locked(void);
static uint8_t Visual_Coast_With_Recovery(uint8_t has_prev, float *prev_x, float *prev_y, float prev_dist, uint8_t target_valid);

// ================== 现有函数 (不变) ==================

void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
}

void Image_Solve(float car_yaw, float *dist, float *angle) {
    extern float uart_data[8];
    // 直接获取无人机解算好的物理坐标 (单位: cm)
    float x_car = uart_data[0];       // [0] 小车地面X坐标
    float y_car = uart_data[1];       // [1] 小车地面Y坐标
    float x_target = uart_data[2];    // [2] 目标地面X坐标
    float y_target = uart_data[3];    // [3] 目标地面Y坐标
    float yaw_drone = uart_data[4];   // [4] 无人机偏航角 (deg)

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
    *angle = atan2f(dy_car, dx_car) * (180.0f / (float)M_PI) + YAW_OFFSET;
}

// ================== 辅助函数 ==================

// 计算跳变检测阈值：prev_car_dist * JUMP_SCALE_COEF 钳位 [JUMP_THRESHOLD_MIN, JUMP_THRESHOLD_MAX] cm
static float Compute_Jump_Threshold(float car_dist) {
    float thr = car_dist * JUMP_SCALE_COEF;
    if (thr < JUMP_THRESHOLD_MIN) thr = JUMP_THRESHOLD_MIN;
    if (thr > JUMP_THRESHOLD_MAX) thr = JUMP_THRESHOLD_MAX;
    return thr;
}

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

    if (track_memory_ms + add_ms > TRACK_MEMORY_MS) {
        track_memory_ms = TRACK_MEMORY_MS;
    } else {
        track_memory_ms += add_ms;
    }
}

static void Visual_Track_Decay(void) {
    if (track_memory_ms > track_memory_step_ms) {
        track_memory_ms -= track_memory_step_ms;
    } else {
        track_memory_ms = 0;
    }
}

static void Visual_Track_Clear(void) {
    track_memory_ms = 0;
    track_memory_last_ms = sys_time_ms;
    track_memory_step_ms = 0;
    track_memory_started = 1;
}

static void Visual_Clear_Soft_Coast(void) {
    visual_coast_end_time = 0;
    visual_coast_expired = 0;
}

static void Visual_Clear_Merge_Coast(void) {
    merge_coast_end_time = 0;
    merge_coast_expired = 0;
}

static uint8_t Visual_Track_Is_Locked(void) {
    return (track_memory_ms >= TRACK_LOCK_THRESHOLD_MS);
}

// 统一软滑行处理（含近距恢复检测），替代原先各处盲回放+到期停车模式
// has_prev: 是否有前一帧信标参考位置
// prev_x, prev_y: 参考信标坐标（若检测到近距目标则更新）
// prev_dist: 前一帧车-信标距离
// 返回 1 表示检测到近距目标已恢复并退出 coast
// target_valid: 本帧信标坐标是否有效。state 1（仅小车可见）时 target_valid=0，
// uart_data[2]/[3] 是冻结的旧坐标，不可用于近距恢复检测，否则会误判"目标重现"→无限循环。
static uint8_t Visual_Coast_With_Recovery(uint8_t has_prev, float *prev_x, float *prev_y, float prev_dist, uint8_t target_valid) {
    // ISR 到期标志：若 ISR 已抢先处理到期的滑行（置位 visual_coast_expired
    // 并清零 visual_coast_end_time），不再重启新滑行周期，避免死循环。
    if (visual_coast_expired) {
        visual_coast_expired = 0;
        Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        return 0;
    }

    if (visual_coast_end_time == 0) {
        visual_coast_end_time = sys_time_ms + COAST_HOLD_MS;
    }
    if (sys_time_ms < visual_coast_end_time) {
        // Coast 活动期：仅当本帧信标确实可见时才做近距恢复检测
        if (has_prev && target_valid) {
            float dx = uart_data[2] - *prev_x;
            float dy = uart_data[3] - *prev_y;
            float dist_prev = sqrtf(dx * dx + dy * dy);
            if (dist_prev <= Compute_Jump_Threshold(prev_dist)) {
                visual_coast_end_time = 0;
                visual_coast_expired = 0;
                *prev_x = uart_data[2];
                *prev_y = uart_data[3];
                return 1;
            }
        }
        // 无近距目标或信标不可见：保持滑行
        Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        return 0;
    } else {
        // Coast 到期(主循环路径)：必须真正停车，而不是继续重放 visual_last。
        // [隐患修复] 旧实现把 visual_coast_end_time 清零后仍下发 visual_last，
        // 下一拍进来发现 end_time==0 会重新拉起一个新的 coast 周期 → "无限续滑"：
        // 主循环这条路永远滑不停，真正刹停只能靠 1ms ISR 的 Visual_Brake_Check，
        // 形成主循环与 ISR 抢时序、滑行忽长忽短、停不干净的脆弱行为。
        // 现在与 ISR 到期路径语义对齐：清零残余速度并停车。即使下一拍 end_time==0
        // 重入，也只会以 0 速度"滑行"，不再驱动电机。
        if (has_prev && target_valid) {
            *prev_x = uart_data[2];
            *prev_y = uart_data[3];
        }
        visual_coast_end_time = 0;
        visual_last_vx = 0.0f;
        visual_last_vy = 0.0f;
        Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        return 0;
    }
}

// ================== 状态处理函数 ==================

// 状态 0：全丢 → 停车 + 清零
static void State0_Handler(void) {
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    Visual_Track_Clear();
    has_prev_target = 0;
    dash_end_time = 0;
    dash_source = 0;
    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    Visual_Clear_Soft_Coast();
    Visual_Clear_Merge_Coast();
}

// 状态 1 或 2：单目标丢失
// locked_state: 1=仅小车可见, 2=仅信标可见 (target_valid 据此区分)
static void State12_Handler(uint8_t locked_state) {
    // 单目标丢失时按真实经过时间衰减锁定置信度，语义等价于旧 valid_track_cnt--。
    Visual_Track_Decay();
    Visual_Clear_Merge_Coast();

    // state1/2 不再启动不可中断 dash，避免信标启动/闪烁造成固定到期刹停。
    Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist, (locked_state == 2));
}

// 状态 3：双目标锁定 (正常追踪)
static void State3_Handler(void) {
    Visual_Clear_Soft_Coast();

    // 跳变滑行到期标记：到期时跳过跳变检测，直接接受新目标位置，打破死循环。
    // merge_coast_expired 由 1ms ISR 置位，解决 ISR 先于主循环清零 merge_coast_end_time
    // 导致 merge_expired 永不为真的竞态。标志仅一帧有效，消费后即清零。
    uint8_t merge_expired = merge_coast_expired || (merge_coast_end_time > 0 && sys_time_ms >= merge_coast_end_time);
    merge_coast_expired = 0;
    uint8_t in_merge = (merge_coast_end_time > 0 && sys_time_ms < merge_coast_end_time);

    if (in_merge) {
        // 滑行期间：检查新目标是否已回到原信标附近，若回归则退出滑行恢复追踪
        float dx = uart_data[2] - prev_target_x;
        float dy = uart_data[3] - prev_target_y;
        float dist_prev = sqrtf(dx * dx + dy * dy);
        float jump_thr = Compute_Jump_Threshold(prev_car_dist);

        if (dist_prev <= jump_thr) {
            // 目标回到原信标附近：退出滑行，恢复追踪
            Visual_Clear_Merge_Coast();
            in_merge = 0;
        } else {
            // 仍是远处信标：维持原方向滑行，prev 不更新，死咬原始信标
            Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
            Visual_Track_Refresh();
            is_edge = (uart_data[7] > EDGE_DIST_CM);
        }
    }

    if (!in_merge) {
        Visual_Clear_Merge_Coast();

        float dist = 0.0f, angle = 0.0f;
        Image_Solve(imu_car_rc_data.yaw, &dist, &angle);

        float car_target_dist = uart_data[7];
        uint8_t jump_detected = 0;

        // merge_coast 到期时跳过跳变检测：直接接受当前信标为新目标
        if (has_prev_target && !merge_expired) {
            float dx = uart_data[2] - prev_target_x;
            float dy = uart_data[3] - prev_target_y;
            float jump = sqrtf(dx * dx + dy * dy);
            float jump_thr = Compute_Jump_Threshold(prev_car_dist);
            // [修复] merge_coast 靠重放 visual_last 速度惯性滑过跳变；若信标交接期间速度
            // 已被 dash/coast 到期的 Visual_State_Reset 清零，则"滑行"会退化为原地死停 400ms
            // (走一下→停一下→继续走)。无残余速度可滑时直接接受新信标，消除该卡顿。
            float coast_speed_sq = visual_last_vx * visual_last_vx + visual_last_vy * visual_last_vy;
            if (jump > jump_thr && coast_speed_sq > 0.01f) {
                jump_detected = 1;
                merge_coast_end_time = sys_time_ms + MERGE_COAST_MS;
            }
        }

        if (jump_detected) {
            Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        } else {
            // 无跳变（含 merge_coast 到期接受新目标）：正常追踪并更新参考坐标

            dist_out = dist;

            float current_speed = TARGET_SPEED;

            float angle_rad = angle * ((float)M_PI / 180.0f);
            float target_speed_x = current_speed * cosf(angle_rad);
            float target_speed_y = current_speed * sinf(angle_rad);

            Mecanum_Set_Velocity(target_speed_x, target_speed_y, 0.0f);

            visual_last_vx = target_speed_x;
            visual_last_vy = target_speed_y;

            prev_target_x = uart_data[2];
            prev_target_y = uart_data[3];
            prev_car_dist = car_target_dist;
            has_prev_target = 1;
        }

        Visual_Track_Refresh();
        is_edge = (uart_data[7] > EDGE_DIST_CM);
    }
}

// 状态 4：发生融合，进入盲冲/滑行判断
static void State4_Handler(void) {
    Visual_Clear_Merge_Coast();

    if (!is_edge) {
        uint8_t is_cooldown = (rush_cooldown_end_time > 0 && sys_time_ms < rush_cooldown_end_time);
        // 在中心丢失，极大可能是近距离融合，执行硬实时绝对精确盲冲 (加入1秒防重入冷却)
        if (dash_end_time == 0 && Visual_Track_Is_Locked() && !is_cooldown) {
            Visual_Clear_Soft_Coast();
            float speed_sq = visual_last_vx * visual_last_vx + visual_last_vy * visual_last_vy;
            float speed = sqrtf(speed_sq);
            if (speed < 0.1f) speed = 0.1f; // 防除零

            // 物理绝对时间换算: 时间(s) = 距离(m) / 速度(m/s)
            float duration_sec = (prev_car_dist / 100.0f) / speed;
            int32_t duration_ms = (int32_t)(duration_sec * 1000.0f) - 150; // 提前刹车，避免冲过信标

            if (duration_ms > 600) duration_ms = 600;
            if (duration_ms < 100) duration_ms = 100;
            duration_ms += STATE4_DASH_EXTRA_MS;
            dash_end_time = sys_time_ms + (uint32_t)duration_ms;
            dash_source = 1;

            Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
            rush_sign = 1;
        } else {
            // 冷却期内或未经历状态3：软滑行（含近距恢复检测）
            Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist, 1);
        }
    } else {
        // 远处误判融合，软滑行（含近距恢复检测）
        Visual_Coast_With_Recovery(has_prev_target, &prev_target_x, &prev_target_y, prev_car_dist, 1);
    }
}

// ================== 公开函数 ==================

// [重构] 抽离核心状态为全局，以便底层定时器与急停函数能强制干预
void Visual_State_Reset(void) {
    dash_end_time = 0;
    dash_source = 0;
    visual_last_vx = 0.0f;
    visual_last_vy = 0.0f;
    visual_coast_end_time = 0;
    merge_coast_end_time = 0;
    merge_coast_expired = 0;
    visual_coast_expired = 0;
    // 注意：不在此函数内清零 rush_cooldown_end_time。
    // rush_cooldown 是 dash 到期时由 1ms ISR 设置的 1 秒冷却期，
    // 其目的是防止 0 速度无限重入。若被一并清零，冷却形同虚设，
    // 信标闪烁时 state 1 可无限触发新一轮盲冲。
}

// ISR 级时间刹车检查：dash/coast/merge 到期硬处理
// 由 Mecanum_Control_Loop (1ms ISR) 调用，作为主循环串口无数据时的最后防线
void Visual_Brake_Check(void) {
    // [最后一道防线] 硬件级绝对时间刹车：如果系统处于盲冲或滑行且绝对时间已到，强行归零指令。
    // 这填补了主循环串口长时间无数据时无法及时刹车的空窗期隐患。
    if (dash_end_time > 0 && sys_time_ms >= dash_end_time) {
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        Visual_State_Reset();
        rush_cooldown_end_time = sys_time_ms + 1000; // [修复] 盲冲结束，强制进入 1 秒冷却，防止 0 速度无限重入
    }
    else if (visual_coast_end_time > 0 && sys_time_ms >= visual_coast_end_time) {
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        Visual_State_Reset();
        // 标志必须在 Visual_State_Reset() 之后设，否则会被其清零
        visual_coast_expired = 1;
    }
    else if (merge_coast_end_time > 0 && sys_time_ms >= merge_coast_end_time) {
        // [修复] merge_coast 到期时仅停车，不调 Visual_State_Reset：保留 prev_target/has_prev_target
        // 供 Visual_Control_Loop 恢复后继续追踪（接受新信标），而非清空全部视觉状态
        target_vel.vx = 0.0f;
        target_vel.vy = 0.0f;
        target_vel.wz = 0.0f;
        merge_coast_expired = 1;    // 置位标志，防 Visual_Control_Loop 竞态重入跳变检测
        merge_coast_end_time = 0;
    }
}

void Visual_Control_Loop(void) {
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

    if (track_memory_ms == 0) {
        is_edge = 0; // [隐患修复 P2.10]: 追踪彻底断开时，清空老旧的边缘记忆
    }

    static uint8_t zero_consecutive = 0;

    // 连续两帧全丢（state 0）：信标确定熄灭，强制中断一切滑行/盲冲
    if ((uint8_t)uart_data[5] == 0) {
        if (++zero_consecutive >= 2) {
            State0_Handler();
            rush_sign = 0;
            return;
        }
    } else {
        zero_consecutive = 0;
    }

    // [隐患修复3]: 绝对物理时钟接管系统。一旦盲冲启动，无视后续一切视觉状态强制执行，直到绝对物理时间到达。
    // 这彻底解决了无人机丢包、相机曝光导致单帧时长被放大所引发的冲刺距离失控问题。
    if (dash_end_time > 0) {
        Mecanum_Set_Velocity(visual_last_vx, visual_last_vy, 0.0f);
        rush_sign = 1;
        return; // 提前退出，屏蔽后续视觉解析！
    }

    // 0: 全丢, 1: 仅小车, 2: 仅信标, 3: 都有, 4: 发生近距离融合（盲冲）
    uint8_t locked_state = (uint8_t)uart_data[5];
    rush_sign = 0;

    if (locked_state == 0) {
        // 仅在全丢时彻底清空历史跟踪记忆；coast 期间保留供跳变检测
        has_prev_target = 0;
        Visual_Clear_Merge_Coast();
    }

    switch (locked_state) {
        case 0:  State0_Handler();  break;
        case 1:  /* fall through */
        case 2:  State12_Handler(locked_state); break;
        case 3:  State3_Handler();  break;
        case 4:  State4_Handler();  break;
        default: break;
    }
}
