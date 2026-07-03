#ifndef _CAR_BOARD_COMM_H
#define _CAR_BOARD_COMM_H

#include "zf_common_headfile.h"

// ================= 板间通讯硬件配置 =================
#define BOARD_UART       UART_1
#define BOARD_BAUDRATE   1000000
#define BOARD_TX_PIN     UART1_TX_P06_1
#define BOARD_RX_PIN     UART1_RX_P06_0

// ================= 外部变量声明 =================
// 供其他文件调用的变量
// uart_data[8] 索引映射 (无人机→小车下传协议):
//   [0] car_ground_pos.x    — 小车地面X坐标 (cm)
//   [1] car_ground_pos.y    — 小车地面Y坐标 (cm)
//   [2] target_ground_pos.x — 目标地面X坐标 (cm)
//   [3] target_ground_pos.y — 目标地面Y坐标 (cm)
//   [4] drone_yaw           — 无人机偏航角 (deg, 顺时针正)
//   [5] locked_state        — 目标锁定状态 (0=全丢/1=仅小车/2=仅信标/3=都有/4=近距离融合盲冲)
//   [6] car_en              — 急停使能标志 (0=急停, 1=正常)
//   [7] car_target_dist     — 车-信标地面距离 (cm)
extern float uart_data[8]; 
extern fifo_struct board_rx_fifo;
extern uint8_t temp_rx_dat;
extern volatile uint32_t board_rx_ok_count;
extern volatile uint32_t board_rx_checksum_fail_count;
extern volatile uint32_t board_rx_tail_fail_count;
extern volatile uint32_t board_rx_invalid_count;
extern volatile uint32_t board_rx_last_dt_ms;
extern volatile uint32_t board_rx_max_dt_ms;
extern volatile uint32_t board_rx_fifo_max_used;
extern volatile uint32_t board_rx_fifo_corrupt_count;

// ================= 函数声明 =================
void Board_Comm_Init(void);               // 通讯初始化

void Parse_Board_Uart_Data(void);         // 正常工作函数
void Debug_Parse_Board_Uart_Data(void);   // 调试专用函数

#endif
