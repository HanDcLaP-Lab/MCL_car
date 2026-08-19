#ifndef _WIRELESS_UART_H
#define _WIRELESS_UART_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>


void wireless_uart_init_();
void wireless_uart_get_();
void wireless_uart_send_int(int32_t send_a);
void wireless_uart_send_float(float send_a);
void wireless_uart_output_motor(void);
void wireless_uart_output_pid(void);
void wireless_uart_output_target(void);
void wireless_uart_output_imu(void);
void wireless_uart_output_encoder(void);
void wireless_uart_output_coast(void);
// [新增] 无人机前馈方向指令 (度)：以无人机yaw=0为0度、顺时针为正，范围[0,360)
void wireless_uart_output_feedforward(float angle_deg);
// 手动调试用：默认不调用，需要时在 PIT_CH1 等调试位置临时打开。
void wireless_uart_output_stop_debug(void);
void wireless_uart_output_comm_debug(void);
void wireless_uart_output_commu(void);
void wireless_uart_check_and_output_stop_reason(void);
void print_imu(void);
#endif
