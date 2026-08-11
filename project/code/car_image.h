#ifndef _CAR_IMAGE_H
#define _CAR_IMAGE_H

#include "zf_common_headfile.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// UART接收的数据数组 uart_data[12] — 完整映射见 car_board_comm.h
// 0: Car X, 1: Car Y (无人机坐标系下的地面坐标, cm)
// 2: Target X, 3: Target Y (无人机坐标系下的地面坐标, cm)
// 4: Drone Yaw (无人机偏航角, deg, 顺时针正)
// 5: locked_state (目标锁定状态: 0=全丢/1=仅小车/2=仅信标/3=都有)
// 6: car_en (急停使能标志, 0=停 1=行)
// 7: car_target_dist (车-信标地面距离, cm)
// 8~11: 第二、第三信标的X/Y坐标 (cm)


void Image_Init(void);
// 解算目标相对于小车的距离和方位角
void Image_Solve(float car_yaw, float *dist, float *angle);

// ================== 视觉跟踪全局变量 (定义在 car_image.c) ==================
extern volatile float    visual_last_vx;
extern volatile float    visual_last_vy;
extern volatile uint32_t dash_end_time;
extern volatile uint32_t rush_cooldown_end_time;
extern int rush_sign;
extern float dist_out;

// ================== 无人机前馈 (定义在 car_image.c) ==================
extern uint32_t feedforward_hesitate_ms; // [新增] 新接受方向犹豫期 (ms)，由宏 FEEDFORWARD_HESITATE_MS 赋初值，可无线调参

// ================== 视觉跟踪 API ==================
/**
 * @brief 视觉控制循环，由主循环在收到无人机视觉数据包后调用
 */
void Visual_Control_Loop(void);

/**
 * @brief 复位所有视觉跟踪状态 (盲冲/滑行/参考坐标等)
 */
void Visual_State_Reset(void);

/**
 * @brief ISR 级时间刹车检查 (盲冲到期处理)
 * @note  由 Mecanum_Control_Loop (1ms ISR) 调用，作为主循环串口无数据时的最后防线
 */
void Visual_Brake_Check(void);

#endif
