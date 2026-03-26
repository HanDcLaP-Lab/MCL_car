#include "car_board_comm.h"
#include "zf_common_headfile.h"

// ================= 变量定义 =================
float uart_data[8] = {0}; 

uint8_t rx_buffer[512];   
volatile uint8_t board_rx_complete_flag = 0;
fifo_struct board_rx_fifo;
uint8_t temp_rx_dat;      

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
    static RxState state = STEP_HEADER1;
    static uint8_t data_idx = 0;
    static FloatPack temp_pack;
    static uint8_t cal_checksum = 0;
    static uint32_t rx_cnt = 0; // [新增] 接收包计数器
    
    uint8_t read_byte;
    uint32_t len;

    while (fifo_used(&board_rx_fifo) > 0) 
    {
        len = 1; 
        fifo_read_buffer(&board_rx_fifo, &read_byte, &len, FIFO_READ_AND_CLEAN);

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
                    state = STEP_HEADER1;                    
                    // 仅在调试模式下打印报错
                    if (debug_en) {
                        //printf("\r\n[ERR] Checksum Fail! Cal:%02X, Rx:%02X\r\n", cal_checksum, read_byte);
                    }
                }
                break;
                
            case STEP_TAIL:
                if (read_byte == 0x7F) {                     
                    // 校验完全通过，赋值
                    for (int i = 0; i < 8; i++) {
                        uart_data[i] = temp_pack.f_data[i];
                    }
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