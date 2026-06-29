/*********************************************************************************************************************
 * car_board_comm.c  小车端(从机) 板间双向通讯
 *
 * 角色: 小车 = 从机(Slave), UART1 + RS485 方向引脚 P06_2。
 * 改造说明: 由原「单向 36 字节协议(0xAA 0x55 + 32 数据 + 校验 + 0x7F, 无 cmd/seq)」
 *           升级为「双向 38 字节协议(含 cmd/seq, 见 board_float8_frame.h)」:
 *   - 接收: 解析无人机下发的 CMD_MASTER 请求帧, 复用 Board_Float8_Frame_Decode 整帧校验,
 *           并保留原有 NaN/Inf 与业务语义防御后写入 uart_data[8]。
 *   - 应答: 收到有效 CMD_MASTER 请求后, 在主循环(非中断)回一帧 CMD_SLAVE 应答(seq 回显),
 *           载荷 car_uplink_data: [0..2]=IMU roll/pitch/yaw(联调观察), [3..7]=0 占位。
 * 接收链路: uart1_isr 收字节入 board_rx_fifo; 主循环 Parse_Board_Uart_Data 排空解析并发送应答。
 ********************************************************************************************************************/
#include "car_board_comm.h"
#include "board_float8_frame.h"
#include "zf_common_headfile.h"
#include "imu_car_rc.h"          // imu_car_rc_data: 小车 IMU 姿态角 (上传载荷来源)

// ================= 变量定义 =================
// 索引映射见 car_board_comm.h 中的 extern 声明注释
float uart_data[8] = {0}; 

// 小车→无人机上传(应答帧)载荷: 槽位 0/1/2 = imu roll/pitch/yaw, 3..7 预留占位 0.0f
// 每次构造应答前由 Car_Board_Send_Reply 刷新为最新 IMU 值
float car_uplink_data[8] = {0};

uint8_t rx_buffer[512];   
volatile uint8_t board_rx_complete_flag = 0;
fifo_struct board_rx_fifo;
uint8_t temp_rx_dat;      
volatile uint32_t board_rx_ok_count = 0;
volatile uint32_t board_rx_checksum_fail_count = 0;
volatile uint32_t board_rx_tail_fail_count = 0;
volatile uint32_t board_rx_invalid_count = 0;
volatile uint32_t board_rx_cmd_mismatch_count = 0;   // [新增] 命令字不匹配计数 (decode 成功但 cmd != CMD_MASTER)
volatile uint32_t board_rx_last_dt_ms = 0;
volatile uint32_t board_rx_max_dt_ms = 0;
volatile uint32_t board_rx_fifo_max_used = 0;
volatile uint8_t  board_rx_last_err = BOARD_ERR_NONE;   // 最近一次接收失败原因码 (调试打印用)

// 接收状态机枚举 (38 字节新协议: 0xAA 0x55 + cmd + seq + 32数据 + 校验 + 0x7F)
// 仅靠帧头 0xAA 0x55 完成重同步, 累积满整帧后交由 Board_Float8_Frame_Decode 整帧校验
typedef enum {
    STEP_HEADER1 = 0,   // 等待帧头第 1 字节 0xAA
    STEP_HEADER2,       // 等待帧头第 2 字节 0x55
    STEP_BODY           // 累积帧体直至凑满 38 字节
} RxState;

// ================= 应答触发标志 =================
// 收到有效 CMD_MASTER 请求帧后需要回复一帧 CMD_SLAVE 应答。
// 解析在 Core_Parse_Board_Uart_Data 中置位下列标志, 由其末尾统一消费并发送应答
// (Car_Board_Send_Reply), 发送放在主循环而非中断里, 避免阻塞中断。
static volatile uint8_t reply_pending = 0;   // 1 = 有一帧待发送应答
static volatile uint8_t reply_echo_seq = 0;  // 待回显给主机的请求帧 seq

// ================= 下传数据防御 (纯函数, 不依赖硬件, 便于宿主机测试) =================
// 对一帧已解码的下传数据 data[8] 做 NaN/Inf 与业务语义防御校验。
// 返回值: 1 = 合法 (可写入 uart_data), 0 = 非法 (应拒绝整帧并保留上一帧)。
// 语义规则迁移自原 36 字节协议:
//   locked_state(data[5]) ∈ [0, 4.5]  (状态只能是 0,1,2,3,4)
//   car_en(data[6])       ∈ [0, 1.5]  (使能只能是 0,1)
//   car_target_dist(data[7]) ∈ [0, 5000] (距离不可能小于 0 或大于 50 米/5000cm)
static uint8_t Board_Downlink_Data_Is_Valid(const float data[8])
{
    // 剔除 NaN (f != f) 与极大异常值(Inf 等)
    for (int i = 0; i < 8; i++) {
        float f = data[i];
        if (f != f || f > 1e6f || f < -1e6f) {
            return 0;
        }
    }

    // 业务语义校验
    float state_f = data[5];
    float en_f    = data[6];
    float dist_f  = data[7];
    if (state_f < 0.0f || state_f > 4.5f)   return 0;   // 状态只能是 0,1,2,3,4
    if (en_f < 0.0f || en_f > 1.5f)         return 0;   // 使能只能是 0,1
    if (dist_f < 0.0f || dist_f > 5000.0f)  return 0;   // 距离 [0, 5000] cm

    return 1;
}

// ================= 整帧处理 (纯函数, 不依赖硬件, 便于宿主机测试) =================
// 对累积满的 38 字节整帧做: 解码 → 命令字判别 → 下传防御 → 写入 uart_data。
// 不在此处发送应答, 仅在收到有效 CMD_MASTER 帧时回填 echo_seq 并返回需要应答标志,
// 由调用者决定何时触发发送 (3.3待补)。
// 参数:
//   frame        : 累积满的 38 字节整帧
//   now_ms       : 当前毫秒时基 (用于接收间隔统计)
//   last_ok_ms   : [in/out] 上一帧成功接收时刻
//   need_reply   : [out] 1 = 收到有效 CMD_MASTER, 需要触发应答
//   echo_seq     : [out] 需要回显的请求帧 seq
// 返回值无意义 (副作用更新全局计数与 uart_data)。
static void Board_Process_Full_Frame(const uint8_t *frame,
                                      uint32_t now_ms,
                                      uint32_t *last_ok_ms,
                                      uint8_t *need_reply,
                                      uint8_t *echo_seq,
                                      uint8_t debug_en)
{
    board_float8_frame_t decoded;

    *need_reply = 0;

    // 整帧解码 (帧头/帧尾/校验和一并校验)
    if (!Board_Float8_Frame_Decode(frame, &decoded)) {
        // 解码失败: 帧头已由状态机保证, 失败必为帧尾或校验和。区分计数以保留原语义。
        if (frame[BOARD_FLOAT8_FRAME_SIZE - 1u] != 0x7F) {
            board_rx_tail_fail_count++;
            board_rx_last_err = BOARD_ERR_TAIL;
        } else {
            board_rx_checksum_fail_count++;
            board_rx_last_err = BOARD_ERR_CHECKSUM;
        }
        if (debug_en) {
            //printf("\r\n[ERR] Decode Fail!\r\n");
        }
        return;
    }

    // 命令字判别: 仅 CMD_MASTER 视为有效下传请求帧
    if (decoded.cmd != BOARD_FLOAT8_CMD_MASTER) {
        board_rx_cmd_mismatch_count++;   // 命令字不匹配, 不应答
        board_rx_last_err = BOARD_ERR_CMD;
        return;
    }

    // 收到有效 CMD_MASTER 请求帧 → 标记需要触发应答 (3.3待补: 实际发送 CMD_SLAVE 应答帧)
    // 注意: 应答与下传数据语义无关, 即便数据被语义防御拒绝, 协议层仍需回 ack 给主机,
    //       否则主机会判超时丢包。此处先标记触发点, 发送逻辑由任务 3.3 实现。
    *need_reply = 1;
    *echo_seq   = decoded.seq;

    // 下传数据防御 (NaN/Inf + 业务语义)
    if (Board_Downlink_Data_Is_Valid(decoded.data)) {
        // 校验通过, 赋值
        for (int i = 0; i < 8; i++) {
            uart_data[i] = decoded.data[i];
        }

        // 接收间隔统计
        if (*last_ok_ms != 0) {
            board_rx_last_dt_ms = now_ms - *last_ok_ms;
            if (board_rx_last_dt_ms > board_rx_max_dt_ms) {
                board_rx_max_dt_ms = board_rx_last_dt_ms;
            }
        }
        *last_ok_ms = now_ms;

        board_rx_ok_count++;
        board_rx_complete_flag = 1;
        board_rx_last_err = BOARD_ERR_NONE;   // 本帧成功, 清除最近错误码

        if (debug_en) {
            //printf("[OK] X:%.2f Y:%.2f seq:%u\r\n", uart_data[0], uart_data[1], decoded.seq);
        }
    } else {
        // 非法帧: 拒绝整帧, 保留上一帧有效 uart_data
        board_rx_invalid_count++;
        board_rx_last_err = BOARD_ERR_INVALID;
    }
}

// ================= 应答发送时序配置 (RS485 半双工方向切换) =================
#define BOARD_DIR_SETUP_US   20u    // 方向切到发送态后的建立时间 (等收发器使能)
#define BOARD_TX_HOLD_US     200u   // 发送后等待移位寄存器排空的 DE 保持时间

// ================= 应答发送 (CMD_SLAVE 应答帧, seq 回显) =================
// 收到有效 CMD_MASTER 请求帧后, 在主循环 (Parse) 中调用本函数发送一帧 CMD_SLAVE 应答。
// 流程:
//   1) 刷新上传载荷 car_uplink_data: 槽位 0/1/2 = 最新 imu roll/pitch/yaw, 3..7 = 0.0f;
//   2) 用 Board_Float8_Frame_Encode 编码应答帧 (seq 回显请求帧 seq, 命令字 CMD_SLAVE);
//   3) RS485 方向切换: 置发送态(HIGH)→建立延时→uart_write_buffer 整帧→保持延时→置回接收态(LOW)。
// 注意: 不在中断内发送, 避免 system_delay_us 阻塞中断 (与现有架构一致)。
static void Car_Board_Send_Reply(uint8_t echo_seq)
{
    uint8_t frame[BOARD_FLOAT8_FRAME_SIZE];

    // 1) 刷新上传载荷: 每次应答前取最新 IMU 姿态角, 其余槽位占位 0.0f
    car_uplink_data[0] = imu_car_rc_data.roll;
    car_uplink_data[1] = imu_car_rc_data.pitch;
    car_uplink_data[2] = imu_car_rc_data.yaw;
    car_uplink_data[3] = 0.0f;
    car_uplink_data[4] = 0.0f;
    car_uplink_data[5] = 0.0f;
    car_uplink_data[6] = 0.0f;
    car_uplink_data[7] = 0.0f;

    // 2) 编码 CMD_SLAVE 应答帧, seq 回显请求帧 seq
    Board_Float8_Frame_Encode(BOARD_FLOAT8_CMD_SLAVE, echo_seq, car_uplink_data, frame);

    // 3) RS485 方向切换并发送
    gpio_set_level(BOARD_DIR_PIN, GPIO_HIGH);             // 置发送态
    system_delay_us(BOARD_DIR_SETUP_US);                 // 建立时间
    uart_write_buffer(BOARD_UART, frame, BOARD_FLOAT8_FRAME_SIZE);
    system_delay_us(BOARD_TX_HOLD_US);                   // 保持时间, 等移位寄存器排空
    gpio_set_level(BOARD_DIR_PIN, GPIO_LOW);             // 置回接收态, 空闲保持接收
}

// ================= 通讯初始化 =================
void Board_Comm_Init(void)
{
    fifo_init(&board_rx_fifo, FIFO_DATA_8BIT, rx_buffer, 512);
    // RS485 方向引脚初始化: 推挽输出, 初始低电平进入接收态 (空闲保持接收)
    gpio_init(BOARD_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
    // [临时-引脚冲突] MOTOR_RB_PWM(TCPWM_CH00_P06_1) 与 UART1_TX_P06_1 抢占同一引脚 P06_1。
    // 双向通讯需要 UART1 TX 发送应答帧, 故暂时注释掉右后电机 PWM 初始化, 避免 P06_1 被复用为 PWM。
    // pwm_init(MOTOR_RB_PWM, 17000, 0);
    uart_rx_interrupt(BOARD_UART, 1); 
}

static void Core_Parse_Board_Uart_Data(uint8_t debug_en)
{
    extern volatile uint32_t sys_time_ms;
    static RxState state = STEP_HEADER1;
    static uint8_t frame_buf[BOARD_FLOAT8_FRAME_SIZE];   // 整帧累积缓冲 (38 字节)
    static uint8_t frame_idx = 0;
    static uint32_t last_ok_time_ms = 0;

    uint8_t read_byte;
    uint32_t len;
    uint32_t fifo_now = fifo_used(&board_rx_fifo);
    if (fifo_now > board_rx_fifo_max_used) {
        board_rx_fifo_max_used = fifo_now;
    }

    while (fifo_used(&board_rx_fifo) > 0) 
    {
        len = 1; 
        fifo_read_buffer(&board_rx_fifo, &read_byte, &len, FIFO_READ_AND_CLEAN);

        switch (state) {
            case STEP_HEADER1:
                // 帧头第 1 字节 0xAA
                if (read_byte == 0xAA) {
                    frame_buf[0] = read_byte;
                    state = STEP_HEADER2;
                }
                break;

            case STEP_HEADER2:
                // 帧头第 2 字节 0x55; 遇 0xAA 保持等待第二头字节; 否则回退重同步
                if (read_byte == 0x55) {
                    frame_buf[1] = read_byte;
                    frame_idx = 2;          // 头 2 字节已就位, 从下标 2 起累积帧体
                    state = STEP_BODY;
                } else if (read_byte != 0xAA) {
                    state = STEP_HEADER1;
                }
                break;

            case STEP_BODY:
                // 累积帧体直至凑满整帧 (含 cmd/seq/数据/校验/帧尾)
                frame_buf[frame_idx++] = read_byte;
                if (frame_idx >= BOARD_FLOAT8_FRAME_SIZE) {
                    uint8_t need_reply = 0;
                    uint8_t echo_seq = 0;

                    // 整帧解码 + 命令字判别 + 下传防御 (纯函数, 无硬件依赖)
                    Board_Process_Full_Frame(frame_buf, sys_time_ms, &last_ok_time_ms,
                                             &need_reply, &echo_seq, debug_en);

                    // ===== 应答触发点 =====
                    // 收到有效 CMD_MASTER 帧 → 置位待应答标志并记录回显 seq;
                    // 实际的 Car_Board_Send_Reply(echo_seq) 与 RS485 方向切换在本函数末尾
                    // 消费 reply_pending 时执行 (主循环内, 不在中断里发送)。
                    if (need_reply) {
                        reply_pending  = 1;
                        reply_echo_seq = echo_seq;
                    }

                    // 整帧处理完毕, 回到帧头重同步
                    state = STEP_HEADER1;
                }
                break;
        }
    }

    // ===== 消费 reply_pending: 发送 CMD_SLAVE 应答帧 (放主循环, 不在中断内) =====
    // 即使下传数据被语义防御拒绝, 只要收到有效 CMD_MASTER 帧 (need_reply 已置位) 仍需回 ack,
    // 否则主机会判超时丢包。应答与下传数据合法性无关。
    if (reply_pending) {
        Car_Board_Send_Reply(reply_echo_seq);   // 构造并发送应答 (含 RS485 方向切换)
        reply_pending = 0;
    }
}

// 正常工作函数（静默接收）
void Parse_Board_Uart_Data(void)
{
    Core_Parse_Board_Uart_Data(0); // 传入 0 关闭打印
}

// 调试专用函数（带打印输出）
void Debug_Parse_Board_Uart_Data(void)
{
    Core_Parse_Board_Uart_Data(1); // 传入 1 开启打印
}
