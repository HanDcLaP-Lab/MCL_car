#ifndef _CAR_BOARD_COMM_H
#define _CAR_BOARD_COMM_H

#include "zf_common_headfile.h"

// ================= 板间通讯硬件配置 =================
#define BOARD_UART       UART_1
#define BOARD_BAUDRATE   115200
#define BOARD_TX_PIN     UART1_TX_P06_1
#define BOARD_RX_PIN     UART1_RX_P06_0
// RS485 半双工方向引脚 (DE/RE 接在一起): 高=发送态, 低=接收态; 空闲保持接收态
#define BOARD_DIR_PIN    P06_2

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

// car_uplink_data[8] 索引映射 (小车→无人机上传协议, 应答帧载荷):
//   [0] imu roll  — 小车横滚角 (deg), 联调观察用
//   [1] imu pitch — 小车俯仰角 (deg), 联调观察用
//   [2] imu yaw   — 小车偏航角 (deg), 联调观察用
//   [3..7] 0.0f   — 预留占位, 语义后续填充
extern float car_uplink_data[8];

extern fifo_struct board_rx_fifo;
extern uint8_t temp_rx_dat;
extern volatile uint32_t board_rx_ok_count;
extern volatile uint32_t board_rx_checksum_fail_count;
extern volatile uint32_t board_rx_tail_fail_count;
extern volatile uint32_t board_rx_invalid_count;
extern volatile uint32_t board_rx_cmd_mismatch_count;   // decode 成功但 cmd != CMD_MASTER 的计数
extern volatile uint32_t board_rx_last_dt_ms;
extern volatile uint32_t board_rx_max_dt_ms;
extern volatile uint32_t board_rx_fifo_max_used;

// 最近一次接收失败的原因码 (供调试打印 wireless_uart_output_board_comm 使用)
//   0 = 无错误(最近一次成功)   1 = 校验和失败   2 = 帧尾失败
//   3 = 数据非法(NaN/语义防御被拒)   4 = 命令字不匹配
#define BOARD_ERR_NONE      0u
#define BOARD_ERR_CHECKSUM  1u
#define BOARD_ERR_TAIL      2u
#define BOARD_ERR_INVALID   3u
#define BOARD_ERR_CMD       4u
extern volatile uint8_t board_rx_last_err;

// ================= 函数声明 =================
void Board_Comm_Init(void);               // 通讯初始化 (含 RS485 方向引脚 P06_2 初始化, 进入接收态)

void Parse_Board_Uart_Data(void);         // 正常工作函数 (解析下传帧; 收到 CMD_MASTER 后回复 CMD_SLAVE 应答帧)
void Debug_Parse_Board_Uart_Data(void);   // 调试专用函数

// 应答相关内部声明 (实际发送函数在任务 3.3 于 .c 中以 static 实现, 此处暂不对外暴露):
//   static void Car_Board_Send_Reply(uint8_t echo_seq);  // 构造并发送 CMD_SLAVE 应答帧 (seq 回显, 载荷取 car_uplink_data)

#endif
