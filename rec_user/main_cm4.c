#include "zf_common_headfile.h"

// ================= 硬件配置 =================
// RS485 接收用 (UART1)
#define RS485_UART       UART_1
#define RS485_BAUDRATE   115200
#define RS485_TX_PIN     UART1_TX_P06_1
#define RS485_RX_PIN     UART1_RX_P06_0

// 电脑通信用 (UART0 - Debug)
#define COMPUTER_UART    UART_0

// ================= 变量定义 =================
uint8 rx_buffer[512];       // 接收缓冲区
fifo_struct rx_fifo;        // FIFO 结构体
uint8 temp_data;            // 临时变量，用于中断接收

// 定义共用体用于解析浮点数
typedef union {
    float f_data[3];        // 对应发送端的 3个float
    uint8_t byte_data[12];  // 对应底层的 12个字节
} FloatUnion;

FloatUnion recv_pack;       // 声明共用体变量
uint8_t recv_count = 0;     // 记录已接收的字节数

// 声明中断处理函数 (供 isr.c 调用)
void my_uart1_handler(void);

int main(void)
{
    // 1. 系统基础初始化
    clock_init(SYSTEM_CLOCK_160M); 
    debug_init(); // 初始化 UART0 (连接电脑)

    // 2. 初始化 FIFO (【修正】：改为 FIFO_DATA_8BIT，因为接收的是 uint8)
    fifo_init(&rx_fifo, FIFO_DATA_8BIT, rx_buffer, 512);

    wireless_uart_init();
    // 3. 初始化 RS485 串口 (UART1)
    uart_init(RS485_UART, RS485_BAUDRATE, RS485_TX_PIN, RS485_RX_PIN);
    
    // 4. 开启 UART1 接收中断
    uart_rx_interrupt(RS485_UART, 1);
    
    // 打印提示
    uart_write_string(COMPUTER_UART, "Start Forwarding RS485(UART1) -> PC(UART0)...\r\n");
    wireless_uart_send_string("Start Forwarding RS485(UART1) -> PC(UART0)...\r\n");
    
    while(true)
    {
        // 5. 主循环：检查 FIFO 是否有数据
        if(fifo_used(&rx_fifo) > 0)
        {
            uint8 dat_byte;
            uint32 len;
            
            // 【修正】：从 FIFO 取出一个字节并存放到临时变量 dat_byte
            fifo_read_buffer(&rx_fifo, &dat_byte, &len, FIFO_READ_AND_CLEAN);
            
            // 将读取到的字节存入共用体的字节数组中
            recv_pack.byte_data[recv_count] = dat_byte;
            recv_count++;
            
            // 当凑齐 12 个字节时 (即 3 个 float 接收完毕)
            if(recv_count >= 12)
            {
                recv_count = 0; // 清零计数器，准备接收下一包
                
                // 此时 recv_pack.f_data[0] 到 [2] 已经是还原好的 float 数据了！
                // 你可以对它们进行转发，例如逐个转发浮点数：
                wireless_uart_send_float(recv_pack.f_data[0]);
                wireless_uart_send_float(recv_pack.f_data[1]);
                wireless_uart_send_float(recv_pack.f_data[2]);
                
                // 如果你想通过调试串口打印到电脑上看效果：
                // printf("Get floats: %.2f, %.2f, %.2f\r\n", 
                //        recv_pack.f_data[0], recv_pack.f_data[1], recv_pack.f_data[2]);
            }
        }
    }
}

// ================= 中断逻辑 =================
// 这个函数在 cm4_isr.c 的 uart1_isr 中被调用
void my_uart1_handler(void)
{
    // 查询 UART1 是否收到数据 (代替了报错的 uart_isr_mask)
    if(uart_query_byte(RS485_UART, &temp_data))
    {
        fifo_write_buffer(&rx_fifo, &temp_data, 1);
    }
}