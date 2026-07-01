#include "car_board_comm.h"
#include "zf_common_headfile.h"

// ================= 变量定义 =================
// 索引映射见 car_board_comm.h 中的 extern 声明注释
float uart_data[8] = {0}; 

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

// 接收状态机枚举
typedef enum {
    STEP_HEADER1 = 0,
    STEP_HEADER2,
    STEP_DATA,
    STEP_CHECKSUM,
    STEP_TAIL
} RxState;

// 定义共用体用于解析
typedef union {
    float f_data[8];
    uint8_t byte_data[32];
} FloatPack;

// ================= 通讯初始化 =================
void Board_Comm_Init(void)
{
    fifo_init(&board_rx_fifo, FIFO_DATA_8BIT, rx_buffer, 512);
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
    pwm_init(MOTOR_RB_PWM, 17000, 0); 
    uart_rx_interrupt(BOARD_UART, 1); 
}

static void Core_Parse_Board_Uart_Data(uint8_t debug_en)
{
    extern volatile uint32_t sys_time_ms;
    static RxState state = STEP_HEADER1;
    static uint8_t data_idx = 0;
    static FloatPack temp_pack;
    static uint8_t cal_checksum = 0;
    static uint32_t rx_cnt = 0; // [新增] 接收包计数器
    static uint32_t last_ok_time_ms = 0;
    
    uint8_t read_byte;
    uint32_t len;
    uint32_t primask;

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
                if (data_idx >= 32) state = STEP_CHECKSUM;
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
                    for (int i = 0; i < 8; i++) {
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
                        
                        if (state_f < 0.0f || state_f > 4.5f) data_valid = 0;       // 状态只能是 0,1,2,3,4
                        if (en_f < 0.0f || en_f > 1.5f) data_valid = 0;             // 使能只能是 0,1
                        if (dist_f < 0.0f || dist_f > 5000.0f) data_valid = 0;      // 距离不可能小于0或大于50米(5000cm)
                    }

                    if (data_valid) {
                        // 校验完全通过，赋值
                        for (int i = 0; i < 8; i++) {
                            uart_data[i] = temp_pack.f_data[i];
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
