#include "car_board_comm.h"
#include "zf_common_headfile.h"

/*********************************************************************************************************************
 * car_board_comm.c  板间通讯 (DUPLEX_SWITCH=1 时小车为从机)
 *
 * DUPLEX_SWITCH = 1 (双向): 协议由原「58 字节单向帧」升级为「带 cmd/seq 的双向帧」。
 *   - 接收: 解析无人机 CMD_MASTER 请求帧 (58 字节), 保留原有 NaN/Inf 与业务语义防御;
 *   - 应答: 收到有效请求后置 reply_pending, 由主循环 Board_Comm_Send_Reply() 回一帧
 *           CMD_SLAVE (22 字节, seq 回显, 载荷 = car_uplink_data)。发送放主循环而非
 *           中断, 避免 DE 保持延时阻塞 ISR。
 *   - FIFO 内积压多个请求时只回最新一帧应答 (与「只保留最新帧」的既有语义一致, 也避免
 *     连续发多帧堵塞总线); 因此主循环卡顿会表现为无人机侧的一次超时统计。
 * DUPLEX_SWITCH = 0 (单向): 维持原 58 字节协议与原接收状态机, 不发送任何数据。
 ********************************************************************************************************************/

// ================= 变量定义 =================
// 索引映射见 car_board_comm.h 中的 extern 声明注释
float uart_data[UART_DATA_LENGTH] = {0};

uint8_t rx_buffer[512];   
volatile uint8_t board_rx_complete_flag = 0;
fifo_struct board_rx_fifo;
uint8_t temp_rx_dat;      
volatile uint32_t board_rx_ok_count = 0;
volatile uint32_t board_rx_checksum_fail_count = 0;
volatile uint32_t board_rx_tail_fail_count = 0;
volatile uint32_t board_rx_invalid_count = 0;
volatile uint32_t board_rx_last_dt_ms = 0;
volatile uint32_t board_rx_max_dt_ms = 0;
volatile uint32_t board_rx_fifo_max_used = 0;
volatile uint32_t board_rx_fifo_corrupt_count = 0;   // [并发加固] fifo_used 越界(size被竞态写坏)被清空的次数
volatile uint32_t board_rx_fifo_write_fail_count = 0;
volatile uint8_t board_rx_stop_pending = 0;
volatile uint32_t board_rx_cmd_mismatch_count = 0;   // [新增] 解码成功但 cmd 不匹配 (含自身应答帧回环)
volatile uint32_t board_tx_reply_count = 0;          // [新增] 已发出的 CMD_SLAVE 应答帧数

#if DUPLEX_SWITCH
// 上行载荷: 联调阶段为特征值 (seq/rx计数/探针常量), 每次构造应答前刷新; 见 Board_Comm_Send_Reply
float car_uplink_data[BOARD_UPLINK_COUNT] = {0};

// 应答触发标志: 解析到有效 CMD_MASTER 后置位, 由主循环统一消费并发送
static volatile uint8_t reply_pending  = 0;   // 1 = 有一帧待发送应答
static volatile uint8_t reply_echo_seq = 0;   // 待回显给主机的请求帧 seq

static uint8_t reply_frame[BOARD_UPLINK_FRAME_SIZE];   // 应答帧发送缓冲
#if BOARD_TX_PREAMBLE_LEN > 0U
// 前导字节缓冲: 内容恒定, 初始化一次即可 (作用见 car_board_comm.h 的宏注释)
static uint8_t preamble_bytes[BOARD_TX_PREAMBLE_LEN];
static uint8_t preamble_initialized = 0;
#endif
#endif

// 接收状态机枚举
#if DUPLEX_SWITCH
// 双向: 仅靠帧头 0xAA 0x55 重同步, 累满 58 字节后整帧校验 (校验含 cmd/seq)
typedef enum {
    STEP_HEADER1 = 0,   // 等待 0xAA
    STEP_HEADER2,       // 等待 0x55
    STEP_BODY           // 累积帧体至 BOARD_DOWNLINK_FRAME_SIZE
} RxState;
#else
typedef enum {
    STEP_HEADER1 = 0,
    STEP_HEADER2,
    STEP_DATA,
    STEP_CHECKSUM,
    STEP_TAIL
} RxState;
#endif

// 定义共用体用于解析
typedef union {
    float f_data[UART_DATA_LENGTH];
    uint8_t byte_data[UART_PAYLOAD_BYTES];
} FloatPack;

// 状态机持久状态提到文件作用域: Board_Comm_Reset_Rx() 需要复位它们 (丢弃积压旧帧)
static RxState state = STEP_HEADER1;
static uint8_t data_idx = 0;
static FloatPack temp_pack;
#if !DUPLEX_SWITCH
// 双向模式改为累满整帧后统一校验, 不再逐字节累加; 声明用条件编译保留而非删除
static uint8_t cal_checksum = 0;
#endif
static uint32_t last_ok_time_ms = 0;
#if DUPLEX_SWITCH
static uint8_t rx_frame[BOARD_DOWNLINK_FRAME_SIZE];   // 下行整帧累积缓冲
#endif

// ================= 通讯初始化 =================
void Board_Comm_Init(void)
{
    // RS485 方向引脚: 空闲保持接收态 (低)
    gpio_init(BOARD_RS485_DIR_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    fifo_init(&board_rx_fifo, FIFO_DATA_8BIT, rx_buffer, 512);
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
    uart_rx_interrupt(BOARD_UART, 1); 

#if DUPLEX_SWITCH && (BOARD_TX_PREAMBLE_LEN > 0U)
    if (!preamble_initialized) {
        for (uint8_t i = 0; i < BOARD_TX_PREAMBLE_LEN; i++) {
            preamble_bytes[i] = BOARD_TX_PREAMBLE_BYTE;
        }
        preamble_initialized = 1;
    }
#endif
}

uint8_t Board_Comm_Consume_Stop_Event(void)
{
    uint8_t stop_pending = board_rx_stop_pending;
    board_rx_stop_pending = 0;
    return stop_pending;
}

// [CR-22] 复位接收通道: 丢弃 FIFO 中积压的旧帧并复位状态机/完成标志。
// 在 IMU 校准结束、主循环长时间阻塞恢复等边界调用, 防止旧包在恢复后被当作新指令执行。
void Board_Comm_Reset_Rx(void)
{
    fifo_clear(&board_rx_fifo);
    state = STEP_HEADER1;
    data_idx = 0;
#if DUPLEX_SWITCH
    // 丢弃积压旧帧时同步撤销待发应答: 对应的请求已被判为过期, 不应再回复
    reply_pending = 0;
#else
    cal_checksum = 0;
#endif
    board_rx_complete_flag = 0;
    board_rx_stop_pending = 0;
}

#if DUPLEX_SWITCH
// ================= 下传数据防御 =================
// 对一帧已解码的下传数据做 NaN/Inf 与业务语义校验。返回 1 = 合法, 0 = 非法(拒绝整帧)。
// 语义规则与原 58 字节协议逐条一致, 仅从状态机内联逻辑抽成独立函数。
static uint8_t Board_Downlink_Data_Is_Valid(const float *data)
{
    // 剔除 NaN (f != f) 与极大异常值 (Inf 等)
    for (int i = 0; i < UART_DATA_LENGTH; i++) {
        float f = data[i];
        if (f != f || f > 1e6f || f < -1e6f) {
            return 0;
        }
    }

    // 业务语义校验
    float state_f = data[5];
    float en_f    = data[6];
    float dist_f  = data[7];
    if (state_f < 0.0f || state_f > 3.5f)  return 0;   // 状态只能是 0,1,2,3
    if (en_f < 0.0f || en_f > 1.5f)        return 0;   // 使能只能是 0,1
    if (dist_f < 0.0f || dist_f > 5000.0f) return 0;   // 距离不可能小于0或大于50米(5000cm)

    return 1;
}

// ================= 下行整帧处理 =================
// 对累满的 58 字节整帧: 校验帧尾与校验和 → 命令字判别 → 数据防御 → 写入 uart_data。
// 帧头已由状态机保证, 故校验失败必为帧尾或校验和。
// 收到有效 CMD_MASTER 即置应答标志: 应答与数据语义无关, 即便数据被防御拒绝也要 ack,
// 否则主机会误判超时丢包。
static void Board_Process_Full_Frame(uint8_t debug_en)
{
    const uint32_t data_bytes = (uint32_t)UART_DATA_LENGTH * UART_FLOAT_BYTES;
    extern volatile uint32_t sys_time_ms;
    uint8_t checksum = 0;
    uint32_t i;

    // --- 帧尾 ---
    if (rx_frame[BOARD_DATA_OFFSET + data_bytes + 1U] != BOARD_TAIL) {
        board_rx_tail_fail_count++;
        if (debug_en) {
            //printf("\r\n[ERR] Tail Fail!\r\n");
        }
        return;
    }

    // --- 校验和 (覆盖 cmd + seq + 数据区) ---
    checksum += rx_frame[BOARD_CMD_OFFSET];
    checksum += rx_frame[BOARD_SEQ_OFFSET];
    for (i = 0; i < data_bytes; i++) {
        checksum += rx_frame[BOARD_DATA_OFFSET + i];
    }
    if (checksum != rx_frame[BOARD_DATA_OFFSET + data_bytes]) {
        board_rx_checksum_fail_count++;
        if (debug_en) {
            //printf("\r\n[ERR] Checksum Fail!\r\n");
        }
        return;
    }

    // --- 命令字判别: 只认主机请求。半双工总线上自身应答帧回环会落在此分支 ---
    if (rx_frame[BOARD_CMD_OFFSET] != BOARD_PEER_CMD) {
        board_rx_cmd_mismatch_count++;
        return;
    }

    // --- 协议层先 ack (与数据合法性无关) ---
    reply_echo_seq = rx_frame[BOARD_SEQ_OFFSET];
    reply_pending  = 1;

    // --- 数据防御 ---
    memcpy(temp_pack.byte_data, &rx_frame[BOARD_DATA_OFFSET], data_bytes);
    if (!Board_Downlink_Data_Is_Valid(temp_pack.f_data)) {
        board_rx_invalid_count++;   // 非法帧: 保留上一帧有效 uart_data
        return;
    }

    for (int k = 0; k < UART_DATA_LENGTH; k++) {
        uart_data[k] = temp_pack.f_data[k];
    }

    uint8_t parsed_running = (temp_pack.f_data[6] >= 0.5f);
    if (!parsed_running) {
        // 同一批次内停止优先，后续 car_en=1 不得覆盖该事件。
        board_rx_stop_pending = 1;
    }

    uint32_t now = sys_time_ms;
    if (last_ok_time_ms != 0) {
        board_rx_last_dt_ms = now - last_ok_time_ms;
        if (board_rx_last_dt_ms > board_rx_max_dt_ms) {
            board_rx_max_dt_ms = board_rx_last_dt_ms;
        }
    }
    last_ok_time_ms = now;
    board_rx_ok_count++;
    board_rx_complete_flag = 1;
}

// ================= 应答发送 =================
// 主循环调用: 若有待应答请求, 刷新上行载荷并发一帧 CMD_SLAVE (seq 回显)。
// 放在主循环而非中断: DE 保持延时 (BOARD_TX_HOLD_US) 不能阻塞 1ms 控制 ISR。
void Board_Comm_Send_Reply(void)
{
    const uint32_t data_bytes = (uint32_t)BOARD_UPLINK_COUNT * UART_FLOAT_BYTES;
    uint8_t  checksum = 0;
    uint8_t  echo_seq;
    uint32_t i;
    uint32_t primask;

    // 原子消费应答标志: uart1_isr 不写这两个变量, 但解析在主循环、清标志也在主循环,
    // 关中断只为与可能的 Board_Comm_Reset_Rx 撤销动作串行化。
    primask = interrupt_global_disable();
    if (!reply_pending) {
        interrupt_global_enable(primask);
        return;
    }
    echo_seq      = reply_echo_seq;
    reply_pending = 0;
    interrupt_global_enable(primask);

    // 刷新上行载荷为最新 IMU 姿态 (后续前馈控制改传小车速度相关量)。
    car_uplink_data[0] = imu_car_data.roll;
    car_uplink_data[1] = imu_car_data.pitch;
    car_uplink_data[2] = imu_car_data.yaw;
    // [新增] 前馈方向角: feedforward_pending=1 (有未确认值) 时发送留存值, 天然含上行丢包重传
    // (每帧重发直到确认); 无人机确认收到 (uart_data[12]≥0.5) 后复位标志并回 0 空闲。
    // 标志在赋值处 (feedforward_direction_send) 置位, 与角度值无关, 0度方向同样可发送。
    // 反馈标志为本帧刚解析的下传值 (解析在主循环先于本函数执行)。
    if (uart_data[12] >= 0.5f) feedforward_pending = 0;   // 无人机确认收到 → 重置
    car_uplink_data[3] = feedforward_pending ? feedforward_deg : 0.0f;

    reply_frame[0] = BOARD_HEADER1;
    reply_frame[1] = BOARD_HEADER2;
    reply_frame[BOARD_CMD_OFFSET] = BOARD_SELF_CMD;
    reply_frame[BOARD_SEQ_OFFSET] = echo_seq;
    // 帧缓冲是 uint8_t 数组, 无 float 对齐前提, 用 memcpy 写入数据区
    memcpy(&reply_frame[BOARD_DATA_OFFSET], car_uplink_data, data_bytes);

    checksum += reply_frame[BOARD_CMD_OFFSET];
    checksum += reply_frame[BOARD_SEQ_OFFSET];
    for (i = 0; i < data_bytes; i++) {
        checksum += reply_frame[BOARD_DATA_OFFSET + i];
    }
    reply_frame[BOARD_DATA_OFFSET + data_bytes]      = checksum;
    reply_frame[BOARD_DATA_OFFSET + data_bytes + 1U] = BOARD_TAIL;

    // RS485 半双工: 拉高 DE 发送 → 等移位完成 → 拉低回接收态
    gpio_high(BOARD_RS485_DIR_PIN);
    system_delay_us(BOARD_DIR_SETUP_US);

#if BOARD_TX_PREAMBLE_LEN > 0U
    // 前导字节: 吸收总线换向期造成的首字节错位, 保护真正的帧头 (原理见 car_board_comm.h)
    uart_write_buffer(BOARD_UART, preamble_bytes, sizeof(preamble_bytes));
#endif

    uart_write_buffer(BOARD_UART, reply_frame, sizeof(reply_frame));

    system_delay_us(BOARD_TX_HOLD_US);
    gpio_low(BOARD_RS485_DIR_PIN);

    board_tx_reply_count++;
}

// ================= 通讯质量打印 (有线 printf) =================
// 由主循环调用, 内部按 BOARD_PRINT_PERIOD_MS 限频。printf 走 UART_0 @115200,
// 与板间通讯 UART1、无线串口 UART2 均不冲突。
//
// 输出格式 (逗号分隔, \r\n 结尾):
//   car   固定前缀, 便于上位机过滤
//   rx    成功接收的下传帧数
//   tx    已发出的应答帧数      → 正常应与 rx 同步; 明显落后说明主循环卡顿吞了应答
//   ck    校验和失败数
//   tail  帧尾失败数
//   inv   语义防御拒收数 (NaN/Inf 或越界)
//   cmd   命令字不匹配数 (含自身应答帧回环)
//   dt    最近两帧成功接收的间隔 (ms)
//   max   接收间隔最大值 (ms)
//   fifo  当前/历史最大 FIFO 占用 (字节)
//
// 注意: printf 为阻塞式 (每字节忙等 TxComplete), 本行约 45 字节 @115200 要占住主循环约 4ms。
//   而主循环卡死看门狗 MAINLOOP_STALL_MS 只有 10ms, 一次打印即占去约四成预算,
//   越线会锁存 DISARM_MAINLOOP_STALL 强制停车 (详见仓库根目录 TOFIX.md 的 P0-1)。
//   故默认不调用 (见 main_cm4.c 注释掉的调用处), 仅在需要观察链路质量时临时开启。
void Board_Comm_Print_Stats(void)
{
    extern volatile uint32_t sys_time_ms;
    static uint32_t last_print_ms = 0;
    uint32_t now = sys_time_ms;

    if ((uint32_t)(now - last_print_ms) < BOARD_PRINT_PERIOD_MS) return;
    last_print_ms = now;

    printf("car,%u,%u,%u,%u,%u,%u,%u,%u,%u/%u\r\n",
           (unsigned)board_rx_ok_count,
           (unsigned)board_tx_reply_count,
           (unsigned)board_rx_checksum_fail_count,
           (unsigned)board_rx_tail_fail_count,
           (unsigned)board_rx_invalid_count,
           (unsigned)board_rx_cmd_mismatch_count,
           (unsigned)board_rx_last_dt_ms,
           (unsigned)board_rx_max_dt_ms,
           (unsigned)fifo_used(&board_rx_fifo),
           (unsigned)board_rx_fifo_max_used);
}
#endif

static void Core_Parse_Board_Uart_Data(uint8_t debug_en)
{
    extern volatile uint32_t sys_time_ms;
#if !DUPLEX_SWITCH
    static uint32_t rx_cnt = 0; // [新增] 接收包计数器
#endif

    uint8_t read_byte;
    uint32_t len;
    uint32_t primask;

    // 以下两个标志描述”本次 FIFO 排空批次”，主循环会在本轮立即消费。
    board_rx_stop_pending = 0;

    // [并发加固] board_rx_fifo 的 fifo->size 被 uart1_isr(写) 与本函数(读) 跨上下文
    // 非原子读改写。竞态丢更新会让 size 漂移；而 fifo_used()=max-size 是无符号运算，
    // 一旦 size 越界导致下溢成巨大值，原来的 while(fifo_used>0) 会退化成死循环，
    // 直接拖死主循环(car_en 处理与通信看门狗都在主循环里) —— 即"停止信号失效"的根源之一。
    // 对策：
    //   ① 关中断内快照一次 fifo_used 并做越界钳位(size 已损坏则清空 FIFO，绝不进死循环)；
    //   ② 用快照长度 fifo_now 驱动循环 → 循环次数有界，永远不可能无限打转；
    //   ③ 每次 fifo_read_buffer 用临界区与写方 ISR 串行化，保护 size 的读改写。
    primask = interrupt_global_disable();
    uint32_t fifo_now = fifo_used(&board_rx_fifo);
    if (fifo_now > sizeof(rx_buffer)) {          // size 被写坏 → fifo_used 下溢，判定损坏
        fifo_clear(&board_rx_fifo);
        fifo_now = 0;
        board_rx_fifo_corrupt_count++;
    }
    interrupt_global_enable(primask);

    if (fifo_now > board_rx_fifo_max_used) {
        board_rx_fifo_max_used = fifo_now;
    }

    while (fifo_now > 0)
    {
        len = 1;
        primask = interrupt_global_disable();
        fifo_read_buffer(&board_rx_fifo, &read_byte, &len, FIFO_READ_AND_CLEAN);
        interrupt_global_enable(primask);
        if (len == 0) break;                     // 防呆：未读到数据立即退出，杜绝空转
        fifo_now--;

#if DUPLEX_SWITCH
        // ---------- 双向: 累满 58 字节整帧后统一校验 (校验含 cmd/seq) ----------
        switch (state) {
            case STEP_HEADER1:
                if (read_byte == BOARD_HEADER1) {
                    rx_frame[0] = read_byte;
                    state = STEP_HEADER2;
                }
                break;

            case STEP_HEADER2:
                if (read_byte == BOARD_HEADER2) {
                    rx_frame[1] = read_byte;
                    data_idx = 2;
                    state = STEP_BODY;
                } else if (read_byte != BOARD_HEADER1) {
                    // 连续 0xAA 时停在 HEADER2 等 0x55, 其余字节退回重新找帧头
                    state = STEP_HEADER1;
                }
                break;

            case STEP_BODY:
                rx_frame[data_idx++] = read_byte;

                // 命令字一到位就先判别: 自身 22 字节应答帧回环时, 按下行 58 字节累积会
                // 读出 cmd=CMD_SLAVE, 此处提前丢弃并重同步, 不必等累满整帧。
                if (data_idx == BOARD_DATA_OFFSET &&
                    rx_frame[BOARD_CMD_OFFSET] != BOARD_PEER_CMD) {
                    board_rx_cmd_mismatch_count++;
                    state = STEP_HEADER1;
                    data_idx = 0;
                    break;
                }

                if (data_idx >= BOARD_DOWNLINK_FRAME_SIZE) {
                    Board_Process_Full_Frame(debug_en);
                    state = STEP_HEADER1;
                    data_idx = 0;
                }
                break;

            default:
                state = STEP_HEADER1;
                data_idx = 0;
                break;
        }
#else
        // ---------- 单向(原实现): 逐字段状态机 ----------
        switch (state) {
            case STEP_HEADER1:
                if (read_byte == 0xAA) state = STEP_HEADER2; 
                break;
                
            case STEP_HEADER2:
                if (read_byte == 0x55) {                     
                    state = STEP_DATA;
                    data_idx = 0;
                    cal_checksum = 0;
                } else if (read_byte != 0xAA) {
                    state = STEP_HEADER1;
                }
                break;
                
            case STEP_DATA:
                temp_pack.byte_data[data_idx++] = read_byte;
                cal_checksum += read_byte;                   
                if (data_idx >= UART_PAYLOAD_BYTES) state = STEP_CHECKSUM;
                break;
                
            case STEP_CHECKSUM:
                if (read_byte == cal_checksum) {
                    state = STEP_TAIL;                       
                } else {
                    board_rx_checksum_fail_count++;
                    state = STEP_HEADER1;                    
                    // 仅在调试模式下打印报错
                    if (debug_en) {
                        //printf("\r\n[ERR] Checksum Fail! Cal:%02X, Rx:%02X\r\n", cal_checksum, read_byte);
                    }
                }
                break;
                
            case STEP_TAIL:
                if (read_byte == 0x7F) {                     
                    // [隐患修复 P3.11]: 增加数据合法性防御校验，防止 NaN/Inf 及离谱数据冲垮控制环
                    uint8_t data_valid = 1;
                    for (int i = 0; i < UART_DATA_LENGTH; i++) {
                        float f = temp_pack.f_data[i];
                        // 剔除 NaN (f != f) 和极大异常值(Inf等)
                        if (f != f || f > 1e6f || f < -1e6f) { 
                            data_valid = 0;
                            break;
                        }
                    }
                    
                    // 业务语义校验
                    if (data_valid) {
                        float state_f = temp_pack.f_data[5];
                        float en_f    = temp_pack.f_data[6];
                        float dist_f  = temp_pack.f_data[7];
                        
                        if (state_f < 0.0f || state_f > 3.5f) data_valid = 0;       // 状态只能是 0,1,2,3
                        if (en_f < 0.0f || en_f > 1.5f) data_valid = 0;             // 使能只能是 0,1
                        if (dist_f < 0.0f || dist_f > 5000.0f) data_valid = 0;      // 距离不可能小于0或大于50米(5000cm)
                    }

                    if (data_valid) {
                        // 校验完全通过，赋值
                        for (int i = 0; i < UART_DATA_LENGTH; i++) {
                            uart_data[i] = temp_pack.f_data[i];
                        }

                        uint8_t parsed_running = (temp_pack.f_data[6] >= 0.5f);
                        if (!parsed_running) {
                            // 同一批次内停止优先，后续 car_en=1 不得覆盖该事件。
                            board_rx_stop_pending = 1;
                        }

                        uint32_t now = sys_time_ms;
                        if (last_ok_time_ms != 0) {
                            board_rx_last_dt_ms = now - last_ok_time_ms;
                            if (board_rx_last_dt_ms > board_rx_max_dt_ms) {
                                board_rx_max_dt_ms = board_rx_last_dt_ms;
                            }
                        }
                        last_ok_time_ms = now;
                        board_rx_ok_count++;
                        board_rx_complete_flag = 1;
                        
                        // 仅在调试模式下打印成功信息
                        if (debug_en) {
                            rx_cnt++;
                            // [优化] 每接收50包打印一次，防止打印太快阻塞CPU
                            if (rx_cnt % 50 == 0) {
                                //printf("RxCnt:%d [OK] X:%.2f Y:%.2f\r\n", rx_cnt, uart_data[0], uart_data[1]);
                            }
                        }
                    } else {
                        board_rx_invalid_count++;
                    }
                } else {
                    board_rx_tail_fail_count++;
                    if (debug_en) {
                        //printf("\r\n[ERR] Tail Fail! Expected:7F, Rx:%02X\r\n", read_byte);
                    }
                }
                state = STEP_HEADER1; 
                break;
        }
#endif
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
