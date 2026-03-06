#include "remote_uart.h"
#include "zf_common_headfile.h"

FloatUnion recv_pack;
uint8_t recv_count = 0;
uint8 rx_buffer[BUFFER_SIZE];       // 接收缓冲区
fifo_struct rx_fifo;                // FIFO 结构体
uint8 temp_data;                    // 临时变量，用于中断接收

void remote_uart_init(void){
    fifo_init(&rx_fifo, FIFO_DATA_8BIT, rx_buffer, BUFFER_SIZE);
    uart_init(RS485_UART, RS485_BAUDRATE, RS485_TX_PIN, RS485_RX_PIN);
    uart_rx_interrupt(RS485_UART, 1);
}

void my_uart1_handler(void)
{
    // 【修改点 1】：改为 while 循环
    // 防止在高波特率下，一次中断进来时寄存器里已经有多个字节，导致遗漏
    while(uart_query_byte(RS485_UART, &temp_data))
    {
        fifo_write_buffer(&rx_fifo, &temp_data, 1);
    }
}

void remote_uart_loop(void){
    // 【修改点 2】：必须改为 while！
    // 只要 FIFO 里有数据，就一次性全部读出来处理掉，绝不能每次只读 1 个字节
    while(fifo_used(&rx_fifo) > 0)
    {
        uint8 dat_byte;
        // 【修改点 3】：务必初始化 len = 1 !!! 
        // 明确告诉 FIFO 每次只读 1 个字节
        uint32 len = 1; 
        
        // 从 FIFO 取出一个字节并存放到临时变量 dat_byte
        fifo_read_buffer(&rx_fifo, &dat_byte, &len, FIFO_READ_AND_CLEAN);
        
        // 将读取到的字节存入共用体的字节数组中
        recv_pack.byte_data[recv_count] = dat_byte;
        recv_count++;
        
        // 判断是否凑齐了一包数据 (8 个 float = 32 个字节)
        if(recv_count >= 4 * DAT_NUM)
        {
            recv_count = 0; // 清零计数器，准备接收下一包
            wireless_uart_send_string("RX_OK: "); 
            wireless_uart_send_float(recv_pack.f_data[0]);
            wireless_uart_send_string("\r\n");
            // ==========================================
            // 数据解析成功！
            // 此时 recv_pack.f_data[0] 到 [7] 就是发送端发来的数据了
            // ==========================================
            
            // 测试代码：你可以在这里取消注释，用无线串口把第一个数据发到电脑看看
            wireless_uart_send_float(recv_pack.f_data[0]);
        }
    }
}