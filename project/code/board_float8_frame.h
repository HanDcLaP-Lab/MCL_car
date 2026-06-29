/*********************************************************************************************************************
 * board_float8_frame.h  无人机 <-> 小车 板间双向通讯协议帧编解码 (小车端副本)
 *
 * 本文件与无人机端 drone/project/code/board_float8_frame.h 字节级完全一致,
 * 两端各保留一份相同实现, 保证主从帧格式严格对齐。源自参考案例
 * 2bl3_wireless_test/project/code/board_float8_frame.*。
 *
 * 帧布局 (共 BOARD_FLOAT8_FRAME_SIZE = 38 字节):
 *   偏移  长度  字段       说明
 *   0     1     header1    固定 0xAA
 *   1     1     header2    固定 0x55
 *   2     1     cmd        命令字: 0x10=主机请求(CMD_MASTER), 0x20=从机应答(CMD_SLAVE)
 *   3     1     seq        序列号 (从机应答时回显主机请求的 seq)
 *   4     32    data[8]    8 个 float, 小端 IEEE754
 *   36    1     checksum   累加校验: 对 cmd+seq+32 字节数据区(共 34 字节)按字节累加, 模 256
 *   37    1     tail       固定 0x7F
 *
 * 小车角色: 从机(Slave) —— 收到 CMD_MASTER 请求后回 CMD_SLAVE 应答帧。
 ********************************************************************************************************************/
#ifndef _BOARD_FLOAT8_FRAME_H_
#define _BOARD_FLOAT8_FRAME_H_

#include <stdint.h>

#define BOARD_FLOAT8_COUNT        8u                                          // 数据载荷 float 个数
#define BOARD_FLOAT8_DATA_BYTES   (BOARD_FLOAT8_COUNT * 4u)                   // 数据区字节数 = 32
#define BOARD_FLOAT8_FRAME_SIZE   (2u + 1u + 1u + BOARD_FLOAT8_DATA_BYTES + 1u + 1u) // 整帧字节数 = 38

#define BOARD_FLOAT8_CMD_MASTER   0x10u   // 命令字: 主机(无人机)请求帧
#define BOARD_FLOAT8_CMD_SLAVE    0x20u   // 命令字: 从机(小车)应答帧

// 解码后的帧内容 (帧头/帧尾/校验已剥离, 仅保留有效字段)
typedef struct {
    uint8_t cmd;                      // 命令字 (CMD_MASTER / CMD_SLAVE)
    uint8_t seq;                      // 序列号
    float data[BOARD_FLOAT8_COUNT];   // 8 个 float 数据载荷
} board_float8_frame_t;

void Board_Float8_Frame_Encode(uint8_t cmd, uint8_t seq, const float data[BOARD_FLOAT8_COUNT], uint8_t frame[BOARD_FLOAT8_FRAME_SIZE]);
uint8_t Board_Float8_Frame_Decode(const uint8_t frame[BOARD_FLOAT8_FRAME_SIZE], board_float8_frame_t *out);

#endif
