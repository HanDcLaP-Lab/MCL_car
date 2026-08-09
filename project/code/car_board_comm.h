#ifndef _CAR_BOARD_COMM_H
#define _CAR_BOARD_COMM_H

#include "zf_common_headfile.h"

// ================= 板间通讯模式编译期开关 =================
// DUPLEX_SWITCH: 板间通讯模式总开关 (编译期生效)
//   1 = 双向主从请求-应答: 解析无人机 CMD_MASTER 请求帧, 回 CMD_SLAVE 应答帧 (小车=从机)
//   0 = 回退到原单向接收 (仅解析 54 字节无 cmd/seq 的旧协议帧)
// 注意: 本开关必须与无人机端 data_complex.h 的 DUPLEX_SWITCH 保持一致, 否则帧格式不匹配。
#define DUPLEX_SWITCH 1

// ================= 板间通讯硬件配置 =================
#define BOARD_UART       UART_1
#define BOARD_BAUDRATE   1000000
#define BOARD_TX_PIN     UART1_TX_P06_1
#define BOARD_RX_PIN     UART1_RX_P06_0
#define BOARD_RS485_DIR_PIN P06_2 // MAX3485 RE#/DE: 低电平为仅接收, 高电平为发送
#define UART_DATA_LENGTH 12U
#define UART_FLOAT_BYTES 4U
#define UART_PAYLOAD_BYTES (UART_DATA_LENGTH * UART_FLOAT_BYTES)
#define TARGET_CANDIDATE_COUNT 3U

// ================= 双向协议帧定义 (与无人机端 duplex_comm.h 逐字段一致) =================
// 帧布局: 0xAA 0x55 + cmd + seq + N×float(小端) + 累加校验(cmd+seq+数据区 模256) + 0x7F
// 下行(无人机→小车) N = UART_DATA_LENGTH   = 12, 整帧 54 字节
// 上行(小车→无人机) N = BOARD_UPLINK_COUNT =  3, 整帧 18 字节
// 两个方向帧长不同, 靠 cmd 区分; 各自状态机只按本方向的帧长累积。
#define BOARD_UPLINK_COUNT   3U     // 上行 float 个数 (测试阶段: IMU roll/pitch/yaw)

#define BOARD_HEADER1        0xAAu
#define BOARD_HEADER2        0x55u
#define BOARD_TAIL           0x7Fu

// 帧内偏移 (两个方向共用): [0]=0xAA [1]=0x55 [2]=cmd [3]=seq [4..]=data
#define BOARD_CMD_OFFSET     2U
#define BOARD_SEQ_OFFSET     3U
#define BOARD_DATA_OFFSET    4U

// 按 float 个数换算整帧长度: 帧头2 + cmd1 + seq1 + 数据区 + 校验1 + 帧尾1
#define BOARD_FRAME_SIZE(n)  (BOARD_DATA_OFFSET + (n) * UART_FLOAT_BYTES + 2U)
#define BOARD_DOWNLINK_FRAME_SIZE  BOARD_FRAME_SIZE(UART_DATA_LENGTH)    // 54
#define BOARD_UPLINK_FRAME_SIZE    BOARD_FRAME_SIZE(BOARD_UPLINK_COUNT)  // 18

#define BOARD_CMD_MASTER     0x10u  // 命令字: 主机(无人机)请求帧
#define BOARD_CMD_SLAVE      0x20u  // 命令字: 从机(小车)应答帧

// 从机视角: 期望收 MASTER, 自己发 SLAVE
#define BOARD_PEER_CMD       BOARD_CMD_MASTER
#define BOARD_SELF_CMD       BOARD_CMD_SLAVE

// RS485 方向引脚时序。
//
// [修复] TX_HOLD 200us → 20us。原理由"末字节可能仍在移位寄存器"不成立:
//   uart_write_byte 每字节都忙等 Cy_SCB_IsTxComplete(), 其语义是「TX FIFO 与移位寄存器
//   双双为空」, 故 uart_write_buffer 返回时最后一位已完整发出, 无需额外保持。
//   多占总线只会延后总线释放, 对下一轮请求不利。20us 仅留时钟抖动与收发器传播延迟余量。
//
// DIR_SETUP: 拉高 DE 到开始发送之间的等待。20us → 100us 是排查上行丢包期间加大的,
//   当时用于把应答发送推后、绕开无人机 DE 多占总线 200us 造成的重叠窗口。
//   现无人机端 TX_HOLD 已治本(200→20), 本项可逐步退回 20us 以省掉这段等待 ——
//   但需实测确认退回后丢包不恶化, 建议一次只改一个参数。
#define BOARD_DIR_SETUP_US   100U
#define BOARD_TX_HOLD_US     20U

// 调试打印周期, 由 Board_Comm_Print_Stats 内部限频。
// printf 是阻塞式的 (每字节忙等 TxComplete), 一行约 49 字节 @115200 要占住主循环约 4.3ms。
// 这里的风险比无人机端更高: 主循环卡死看门狗 MAINLOOP_STALL_MS 只有 10ms, 打印一次就吃掉
// 近一半预算; 一旦总耗时越线, 1ms ISR 会锁存 DISARM_MAINLOOP_STALL 强制停车。
// 联调期取 5000ms 以降低触发概率。详见仓库根目录 TOFIX.md。
#define BOARD_PRINT_PERIOD_MS 5000U

// ================= 应答帧前导字节 =================
// 背景: RS485 总线当前只有 120Ω 终端、无 fail-safe 偏置电阻。无人机发完请求拉低 DE 后,
//   到小车拉高 DE 开始应答之间有约 0.5~2ms 的窗口, 此时两端都不驱动, 差分对处于浮空态。
//   浮空电平漂移可能被无人机接收器判成假 start bit, 导致应答首字节位边界错位 ——
//   实测收到 0x75 而非 0xAA (0x75 与 0xAA 呈移位关系, 是起始位判错的典型特征),
//   帧头一丢, 后续 17 字节找不到 AA 55 就被静默丢弃, 整帧丢失且不计入任何错误计数器。
//
// 对策: 真正的帧头之前先发若干前导字节, 专门用来"喂"给可能错位的接收器。
//   这些字节即便被解成垃圾也无害 —— 无人机状态机在 HEADER1 状态本就丢弃非 0xAA 字节。
//   等总线稳定下来, 真正的 AA 55 才发出。
// 代价: 每帧多 BOARD_TX_PREAMBLE_LEN 字节, @1Mbps 每字节仅 10us。
//
// 实测效果 (2026-08-09): 3 字节前导使上行丢包 4.48%→3.06%; 加大到 8 字节并配合
//   DIR_SETUP 20→100us 后进一步降到 0.31%。
//   但事后定位到真因是无人机端 DUPLEX_TX_HOLD_US=200us 白占总线, 与应答窗口重叠;
//   前导与 DIR_SETUP 的加大属于"从另一头绕过重叠", 并非治本。
//   无人机端 TX_HOLD 已改为 20us 治本, 因此本项可逐步退回 2~3 字节, 每帧省 5~6 字节。
//   退回时需实测确认丢包不恶化, 建议一次只改一个参数。
// 前导字节取值可试两种, 二者物理含义不同:
//   0x00 = 线上"start低 + 8个低 + stop高", 让总线尽量长时间处于低电平;
//   0xFF = 线上"start低 + 8个高 + stop高", 让总线尽早充电到高电平(UART空闲态),
//          若问题源于上升沿 RC 充电不足, 0xFF 可能比 0x00 更有效。
// 设为 0 可关闭本机制做 A/B 对比。
// 注意: 改动本处后, 无人机端 duplex_comm.h 的 DUPLEX_PEER_PREAMBLE_LEN / BYTE 需同步,
//       否则诊断字段 hdrx/ld 会算错 (只影响读数, 不影响通讯本身)。
#define BOARD_TX_PREAMBLE_LEN     8U
#define BOARD_TX_PREAMBLE_BYTE    0x00u

// ================= 外部变量声明 =================
// 供其他文件调用的变量
// uart_data[12] 索引映射 (无人机→小车下传协议):
//   [0] car_body_pos.x      — 小车相对无人机的机体系X坐标 (cm)
//   [1] car_body_pos.y      — 小车相对无人机的机体系Y坐标 (cm)
//   [2] target_body_pos.x   — 目标相对无人机的机体系X坐标 (cm)
//   [3] target_body_pos.y   — 目标相对无人机的机体系Y坐标 (cm)
//   [4] drone_yaw           — 无人机在小车固定地面系中的偏航角 (deg, 顺时针正)
//   [5] locked_state        — 目标锁定状态 (0=全丢/1=仅小车/2=仅信标/3=都有)
//   [6] car_en              — 急停使能标志 (0=急停, 1=正常)
//   [7] car_target_dist     — 车-信标地面距离 (cm)
//   [8] target2_body_pos.x  — 第二信标相对无人机的机体系X坐标 (cm)
//   [9] target2_body_pos.y  — 第二信标相对无人机的机体系Y坐标 (cm)
//   [10] target3_body_pos.x — 第三信标相对无人机的机体系X坐标 (cm)
//   [11] target3_body_pos.y — 第三信标相对无人机的机体系Y坐标 (cm)
extern float uart_data[UART_DATA_LENGTH];

// car_uplink_data[3] 索引映射 (小车→无人机上传协议, 应答帧载荷):
//   [0] imu roll  — 小车横滚角 (deg)
//   [1] imu pitch — 小车俯仰角 (deg)
//   [2] imu yaw   — 小车偏航角 (deg)
// 测试阶段仅供无人机侧观察通讯质量; 后续前馈控制改传小车速度相关量。
// 每次构造应答帧前由 Board_Comm_Send_Reply 刷新为最新 IMU 值。
extern float car_uplink_data[BOARD_UPLINK_COUNT];

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
extern volatile uint32_t board_rx_fifo_write_fail_count;
extern volatile uint8_t board_rx_stop_pending;
extern volatile uint32_t board_rx_cmd_mismatch_count;   // 解码成功但 cmd 不匹配 (含自身应答回环)
extern volatile uint32_t board_tx_reply_count;          // 已发出的 CMD_SLAVE 应答帧数

// 说明: 8.9a 调试期曾有一组诊断埋点 (DE 引脚回读实测电平、累计发送字节数), 用于证明
//   "应答确实驱动了总线"而非只是 CPU 调了 uart_write_buffer。链路定位完成后已移除,
//   详见 README 8.9a。若日后上行再度异常, 从 8.9a 提交取回即可。

// ================= 函数声明 =================
void Board_Comm_Init(void);               // 通讯初始化 (含 RS485 方向引脚初始化, 进入接收态)

void Parse_Board_Uart_Data(void);         // 正常工作函数
void Debug_Parse_Board_Uart_Data(void);   // 调试专用函数
uint8_t Board_Comm_Consume_Stop_Event(void);   // 消费本批次 car_en=0 锁存
void Board_Comm_Reset_Rx(void);           // 丢弃积压旧帧并复位接收状态机 (校准结束/恢复边界用)
#if DUPLEX_SWITCH
void Board_Comm_Send_Reply(void);         // 主循环调用: 若有待应答请求则发一帧 CMD_SLAVE
void Board_Comm_Print_Stats(void);        // 有线 printf 输出收发统计 (主循环调用, 内部限频)
#endif

#endif
