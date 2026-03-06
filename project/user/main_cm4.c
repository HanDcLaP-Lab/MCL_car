
#include "zf_common_headfile.h"

// ================= 板间通讯硬件配置 =================
// 假设使用 UART1 与 CM7_1 进行通讯 (请根据实际接线修改引脚和串口号)
#define BOARD_UART       UART_1
#define BOARD_BAUDRATE   115200
#define BOARD_TX_PIN     UART1_TX_P06_1
#define BOARD_RX_PIN     UART1_RX_P06_0

// ================= 变量定义 =================
float uart_data[8] = {0}; // 你原有的数据数组

uint8_t rx_buffer[512];   // 串口接收 FIFO 缓冲区
fifo_struct board_rx_fifo;
uint8_t temp_rx_dat;      // 用于中断接收的临时变量

// 接收状态机枚举
typedef enum {
    STEP_HEADER1 = 0,
    STEP_HEADER2,
    STEP_DATA,
    STEP_CHECKSUM,
    STEP_TAIL
} RxState;

// 定义共用体用于解析 32 字节 -> 8 个      float
typedef union {
    float f_data[8];
    uint8_t byte_data[32];
} FloatPack;

// ================= 状态机解析函数 =================
void Parse_Board_Uart_Data(void)
{
    static RxState state = STEP_HEADER1;
    static uint8_t data_idx = 0;
    static FloatPack temp_pack;
    static uint8_t cal_checksum = 0;
    
    uint8_t read_byte;
    uint32_t len;

    // 当 FIFO 中有数据时，持续读出并解析
    while (fifo_used(&board_rx_fifo) > 0) 
    {
        len = 1; // 【极其关键的修复】：强制指定只读取 1 个字节！
        fifo_read_buffer(&board_rx_fifo, &read_byte, &len, FIFO_READ_AND_CLEAN);
        
        // （你可以暂时打开这里的注释，看看收到的裸数据）
        // printf("%02X ", read_byte); 

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
                    printf("\r\n[ERR] Checksum Fail! Cal:%02X, Rx:%02X\r\n", cal_checksum, read_byte);
                }
                break;
                
            case STEP_TAIL:
                if (read_byte == 0x7F) {                     
                    // 校验完全通过
                    for (int i = 0; i < 8; i++) {
                        uart_data[i] = temp_pack.f_data[i];
                    }
                    printf("\r\n[5] SUCCESS! float[0]:%.2f, float[1]:%.2f\r\n", uart_data[0], uart_data[1]);
                } else {
                    printf("\r\n[ERR] Tail Fail! Expected:7F, Rx:%02X\r\n", read_byte);
                }
                state = STEP_HEADER1; 
                break;
        }
    }
}

void Wireless_Update(uint8_t ch, float val);

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      
    debug_init();                       

    // --- 1. 初始化板间通讯的 FIFO 和 UART ---
    fifo_init(&board_rx_fifo, FIFO_DATA_8BIT, rx_buffer, 512);
    uart_init(BOARD_UART, BOARD_BAUDRATE, BOARD_TX_PIN, BOARD_RX_PIN);
    uart_rx_interrupt(BOARD_UART, 1); // 开启接收中断

    IMU_Car_Init();
    Encoder_Init();
    Mecanum_Init();
    wireless_uart_init_();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    
    pit_ms_init(PIT_CH1, 400);
    pit_ms_init(PIT_CH2, 20); 
    printf("INIT");
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    pit_ms_init(PIT_CH0, 1);
    system_delay_ms(3000);

    test_program_1();
    
    for(;;)
    {
        // --- 2. 在主循环中不断调用数据解析函数 ---
        static uint32_t print_cnt = 0;
        if(print_cnt++ % 100 == 0) {
            printf("."); 
        }
        Parse_Board_Uart_Data();

        seekfree_assistant_data_analysis();

        for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
            if (seekfree_assistant_parameter_update_flag[i]) {
                seekfree_assistant_parameter_update_flag[i] = 0;
                Wireless_Update(i + 1, seekfree_assistant_parameter[i]); 
                wireless_uart_send_string("Param Updated\r\n");
            }
        }

        system_delay_ms(10); 
        
        pid_lf.kp = KP; pid_lf.ki = KI; pid_lf.kd = KD; pid_lf.max_i = MAX_I;
        pid_rf.kp = KP; pid_rf.ki = KI; pid_rf.kd = KD; pid_rf.max_i = MAX_I;
        pid_lb.kp = KP; pid_lb.ki = KI; pid_lb.kd = KD; pid_lb.max_i = MAX_I;
        pid_rb.kp = KP; pid_rb.ki = KI; pid_rb.kd = KD; pid_rb.max_i = MAX_I;
    }
}





void Wireless_Update(uint8_t ch, float val) {
    switch (ch) {
        case 1:
            KP = val;
            break;
        case 2:
            KI = val;
            break;
        case 3:
            KD = val;
            break;
        case 4:
            MAX_I = val;
            break;
        case 8:
            if(val == 1){
              Mecanum_Stop();}
            if(val== 0){
                wireless_uart_send_string("done");
                Mecanum_Unlock();
            }
        default:
            break;
    }
}