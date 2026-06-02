#ifndef _CAR_BOARD_COMM_H
#define _CAR_BOARD_COMM_H

#include "zf_common_headfile.h"

// ================= 板间通讯硬件配置 =================
#define BOARD_UART       UART_1
#define BOARD_BAUDRATE   115200
#define BOARD_TX_PIN     UART1_TX_P06_1
#define BOARD_RX_PIN     UART1_RX_P06_0

// ================= 外部变量声明 =================
// uart_data[8] 内部接收缓冲区，外部代码请使用 car_stat_t car (定义于 car_image.h)
// 索引映射 (无人机→小车下传协议):
//   [0] car_x, [1] car_y, [2] target_x, [3] target_y
//   [4] drone_yaw, [5] locked_state, [6] car_en, [7] reserved
extern float uart_data[8];
extern fifo_struct board_rx_fifo;
extern uint8_t temp_rx_dat;

// ================= 函数声明 =================
void Board_Comm_Init(void);               // 通讯初始化

void Parse_Board_Uart_Data(void);         // 正常工作函数
void Debug_Parse_Board_Uart_Data(void);   // 调试专用函数

#endif