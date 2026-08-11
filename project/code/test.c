#include "test.h"
#include "zf_common_headfile.h"
#include <math.h>\1

/**
 * ================== 非阻塞测试框架 ==================
 * [新增] 测试逻辑已并入 main 主循环统一调度 (见 main_cm4.c):
 *   - 每个测试是非阻塞状态机, 由 Test_Execute() 每轮主循环调用一次 (~1ms),
 *     不再占用独立 while(1), 因此主循环的板间解析/急停处理全程生效;
 *   - 测试不依赖无人机下传: 无帧时 DISARM_COMM_LOST 在测试模式不生效 (见 main_cm4.c),
 *     车辆可脱离无人机在台架上正常测试;
 *   - 无人机下传 car_en=0 时, 底盘由 1ms ISR 立即断电停车 (DISARM_DRONE_STOPPED),
 *     测试检测到未解锁即冻结 (测试代码无任何解锁/恢复逻辑);
 *     car_en=1 (无人机高度足够且未急停) 时底盘自动解锁, 测试从当前阶段继续;
 *   - 无线 ch8 人工急停 (DISARM_MANUAL) 仍生效: ch8=1 停车, ch8=0 恢复;
 *     旧版 test_program_1 入口强制解除该位, 新版不再强制;
 *   - 无线调参由主循环统一处理, 不再需要本文件内的重复处理函数。
 */

// ================== 测试参数 (可自行调整) ==================
#define TEST_A_SPEED_MPS   0.5f    // A: 前进/后退速度 (m/s) —— 原 test_program_1 行为
#define TEST_A_TIME_MS     3000U   // A: 单相持续时间 (ms)
#define TEST_B_SPEED_MPS   1.1f    // B: 前进/后退速度 (m/s)
#define TEST_B_DIST_M      3.0f    // B: 单相目标距离 (m, 编码器里程积分)
#define TEST_B_TIMEOUT_MS  7000U   // B: 单相超时保护 (堵轮/架空卡死检测, 0=关闭)
// C: 开环PWM占空比 (绝对值和方向), 四电机独立赋值; 需要单独测某轮时把其余轮置 0
#define TEST_C_PWM_LF      0.0f  // 左前
#define TEST_C_PWM_RF      3000.0f  // 右前
#define TEST_C_PWM_LB      3000.0f  // 左后
#define TEST_C_PWM_RB      0.0f  // 右后
#define TEST_C_TIME_MS     20000U   // C: 开环持续时间 (ms), 单次运行后停止
#define ACTIVE_TEST        TEST_SELECT_B  // 选择当前生效测试: TEST_SELECT_A / B / C

// ================== 测试选择与共享上下文 ==================
// 测试选择值必须用 #define 而非 enum: 预处理器不识别枚举常量,
// #if 中未定义标识符一律按 0 处理 → 曾导致 #if ACTIVE_TEST == TEST_SELECT_A 恒真,
// 无论 ACTIVE_TEST 选哪个测试, 永远执行测试A。
#define TEST_SELECT_A  0   // 定时往返 (原 test_program_1 行为, 循环)
#define TEST_SELECT_B  1   // 定距往返 (编码器里程积分, 循环)
#define TEST_SELECT_C  2   // 开环PWM单次运行

typedef struct {
    uint8_t  started;        // 首次执行标志: 首轮建立计时/里程基准 (防 boot 后大时间偏移误翻相)
    uint8_t  phase;          // 当前阶段号 (各测试自行定义阶段语义)
    uint32_t phase_start_ms; // 阶段起始时间 (时间等待/超时基准)
    uint32_t last_ms;        // 上一轮时间戳 (里程积分 dt 基准)
    float    dist_m;         // 本阶段累积里程 (m, 带符号)
} Test_Ctx_t;

// ================== 非阻塞等待原语 ==================

// 每个测试入口统一调用 Test_Entry: 未解锁(car_en=0/ch8)返回0冻结, 不推进计时/里程。
// car_en=1 或 ch8 释放后底盘解锁, 测试从当前阶段继续 (阶段计时含停止间隔, 无实质影响)。
// 首次执行建立计时/里程基准: 否则 ctx 全零时 sys_time_ms 已累计数秒,
// 首个 Test_Wait_Time 会误判到期立即翻相 (首相被跳过)。
static uint8_t Test_Entry(Test_Ctx_t *ctx)
{
    if (!Chassis_Is_Armed()) return 0;
    if (!ctx->started) {
        ctx->started = 1;
        ctx->phase_start_ms = sys_time_ms;
        ctx->last_ms = sys_time_ms;
    }
    return 1;
}

// [新增] 非阻塞延时: 到期返回1并自动重置基准 (连续调用即周期计时, uint32 差值回绕安全)
static uint8_t Test_Wait_Time(uint32_t *start_ms, uint32_t duration_ms)
{
    if ((uint32_t)(sys_time_ms - *start_ms) >= duration_ms) {
        *start_ms = sys_time_ms;
        return 1;
    }
    return 0;
}

// [新增] 非阻塞里程等待: 左右前轮速度均值 × 真实dt 积分, 累计里程绝对值 ≥ 目标返回1。
// 均值抵消原地自转 (lf/rf 符号相反); 带符号累积 = 净轮位移 (被推回不算进度);
// fabsf 比较兼容正负阶段, 一个原语同时服务前进/后退两个方向。
static uint8_t Test_Wait_Distance(Test_Ctx_t *ctx, float target_m)
{
    uint32_t now = sys_time_ms;
    uint32_t dt_ms = (uint32_t)(now - ctx->last_ms);
    ctx->last_ms = now;
    if (dt_ms > 0U) {
        float v_avg = (encoder_data.lf + encoder_data.rf) * 0.5f;
        ctx->dist_m += v_avg * ((float)dt_ms * 0.001f);
    }
    return fabsf(ctx->dist_m) >= target_m;
}

// ================== 测试A: 定速度+定时间 (前后循环) ==================
// [新增] 复刻原 test_program_1 行为 (0.8m/s 前进3s → 后退3s → 循环), 改为非阻塞状态机
static void Test_Program_A(void)
{
    static Test_Ctx_t ctx = {0};
    if (!Test_Entry(&ctx)) return;                    // 锁车(急停/ch8): 冻结; 首轮建立基准
    switch (ctx.phase) {
        case 0: // 前进
            Mecanum_Set_Velocity(0.0f, TEST_A_SPEED_MPS, 0.0f);
            if (Test_Wait_Time(&ctx.phase_start_ms, TEST_A_TIME_MS)) ctx.phase = 1;
            break;
        case 1: // 后退
            Mecanum_Set_Velocity(0.0f, -TEST_A_SPEED_MPS, 0.0f);
            if (Test_Wait_Time(&ctx.phase_start_ms, TEST_A_TIME_MS)) ctx.phase = 0;
            break;
    }
}

// ================== 测试B: 定速度+定距离 (前后循环) ==================
// [新增] 以编码器里程积分控制切换点: 前进目标距离 → 后退目标距离 → 循环
static void Test_Program_B(void)
{
    static Test_Ctx_t ctx = {0};
    if (!Test_Entry(&ctx)) return;                    // 锁车(急停/ch8): 冻结; 首轮建立基准
    Mecanum_Set_Velocity(ctx.phase == 0 ? TEST_B_SPEED_MPS : -TEST_B_SPEED_MPS, 0.0f, 0.0f);
    // 里程到位或超时(卡死保护) → 翻相
    if (Test_Wait_Distance(&ctx, TEST_B_DIST_M)
        || (TEST_B_TIMEOUT_MS > 0U && Test_Wait_Time(&ctx.phase_start_ms, TEST_B_TIMEOUT_MS))) {
        ctx.phase = 1 - ctx.phase;
        ctx.phase_start_ms = sys_time_ms;
        ctx.dist_m = 0.0f;
        ctx.last_ms = sys_time_ms;
    }
}

// ================== 测试C: 定PWM+定时间 (单次运行) ==================
// [新增] 开环直驱: 直接输出固定PWM占空比 (绕过斜坡/偏航/轮速PID), 时间到即永久停止。
// 注意: 开环无斜坡, 恢复运行瞬间 0 → 满占空比硬起步 (急停后不恢复, 仅 ch8 释放会继续);
//       如需软启动可在状态机内自行加斜坡。
static void Test_Program_C(void)
{
    static Test_Ctx_t ctx = {0};
    if (!Test_Entry(&ctx)) return;                    // 锁车(急停/ch8): 冻结 (ISR !armed 路径已灭PWM); 首轮建立基准
    switch (ctx.phase) {
        case 0: // 开环前进 (单次运行)
            Mecanum_Set_PWM_Open_Loop(TEST_C_PWM_LF, TEST_C_PWM_RF, TEST_C_PWM_LB, TEST_C_PWM_RB);
            if (Test_Wait_Time(&ctx.phase_start_ms, TEST_C_TIME_MS)) ctx.phase = 1;
            break;
        case 1: // 结束: 关开环 + 目标归零, 静止保持 (单次运行完成后不再动作)
            Mecanum_Set_PWM_Open_Loop_Off();
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
            break;
    }
}

// ================== 测试执行入口 ==================
// [新增] 由 main 主循环每轮调用一次 (TEST_MODE != NORMAL 时); 切换测试修改顶部 ACTIVE_TEST 宏
void Test_Execute(void)
{
#if ACTIVE_TEST == TEST_SELECT_A
    Test_Program_A();
#elif ACTIVE_TEST == TEST_SELECT_B
    Test_Program_B();
#elif ACTIVE_TEST == TEST_SELECT_C
    Test_Program_C();
#endif
}

// ================== IMU 测试 (保持不变) ==================
void test_program_imu(void)
{
    uint32_t print_time_ms = sys_time_ms;

    Chassis_Block(DISARM_MANUAL);
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);

    while(1){
        IMU_Car_Update_Loop();
        if ((uint32_t)(sys_time_ms - print_time_ms) >= 500U) {
            print_time_ms = sys_time_ms;
            print_imu();
        }
        system_delay_ms(1);
    }
}
