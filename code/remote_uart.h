#ifndef _REMOTE_UART_H
#define _REMOTE_UART_H

#include "zf_common_headfile.h"
/*这里是接收端
recv_pack.f_data是接收到的float数据
*/

// ================= 硬件配置 =================

// RS485 接收用 (UART1)
#define RS485_UART       UART_1
#define RS485_BAUDRATE   115200
#define RS485_TX_PIN     UART1_TX_P06_1
#define RS485_RX_PIN     UART1_RX_P06_0

#define DAT_NUM 3 //传输数据的数量，默认为float
#define BUFFER_SIZE 512 //缓存区大小

extern uint8 rx_buffer[BUFFER_SIZE];       // 接收缓冲区
extern fifo_struct rx_fifo;        // FIFO 结构体
extern uint8 temp_data;            // 临时变量，用于中断接收

typedef union {
    float f_data[DAT_NUM];        // 对应发送端的 3个float
    uint8_t byte_data[( 4 * DAT_NUM )];  // 对应底层的 12个字节
} FloatUnion;

extern FloatUnion recv_pack;
extern uint8_t recv_count;

void my_uart1_handler(void);
void remote_uart_init(void);
void remote_uart_loop(void);
#endif