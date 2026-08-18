/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
extern volatile uint8_t board_rx_complete_flag;
extern int32_t cnt;
extern volatile uint32_t sys_time_ms;
extern volatile uint32_t main_loop_heartbeat_ms;   // 主循环存活心跳 (定义在 mecnum.c)
volatile uint32_t drone_timeout_debug = 0;
volatile uint32_t board_rx_ok_debug = 0;
volatile uint32_t visual_loop_count_debug = 0;
volatile uint32_t visual_loop_dt_debug = 0;
volatile uint32_t visual_loop_max_dt_debug = 0;
volatile uint8_t  board_comm_debug_pending = 0;   // [移出中断] pit0_ch1(200ms) 置位，主循环消费并发送调试打印
//int mv_en = 0;
void Wireless_Update(uint8_t ch, float val);
int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(1500);
    Board_Comm_Init();

    IMU_Car_Init();
    Encoder_Init();
    Mecanum_Init();
    if (TEST_MODE == TEST_MODE_IMU) {
        Chassis_Block(DISARM_MANUAL);
    }
    wireless_uart_init_();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    printf("seekfree init done");
    wireless_uart_send_string("\r\n=== CAR SYSTEM BOOT ===\r\n");
    wireless_uart_send_string("Calibrating IMU, keep car still...\r\n");
    
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    pit_ms_init(PIT_CH0, 1);
    while (imu_car_data.is_calibrated == 0) {
        main_loop_heartbeat_ms = sys_time_ms;   // 存活心跳：校准等待期也喂狗，防止刚武装即误判卡死
        if (TEST_MODE == TEST_MODE_IMU) {
            IMU_Car_Update_Loop();
        } else {
            Parse_Board_Uart_Data();
            seekfree_assistant_data_analysis();
        }
        system_delay_ms(1);
    }
    printf("imu init done");
    wireless_uart_send_string("IMU Calib OK! Entering test wait (3s)...\r\n");
    // 底盘解锁由 1ms ISR 在 IMU 校准完成时自动处理 (Chassis_Unblock(DISARM_UNCALIBRATED))

    // [CR-22] 校准结束边界: 丢弃校准期间积压的旧帧与完成标志, 只允许校准完成后到达的新帧参与控制
    Board_Comm_Reset_Rx();

    if (TEST_MODE == TEST_MODE_TEST) {
        printf("enter waiting");    // 进场等待已移至主循环内非阻塞门控 (见 TEST_ENTRY_WAIT_MS)
    } else if (TEST_MODE == TEST_MODE_IMU) {
        test_program_imu();
    }
    uint32_t last_drone_rx_time_ms = sys_time_ms;
    // 此处编写用户代码 例如外设初始化代码等
    for(;;)
    {
        // 主循环存活心跳：每轮无条件刷新。1ms ISR 若发现超过 MAINLOOP_STALL_MS 未刷新，
        // 判定主循环卡死并强制切断动力。
        main_loop_heartbeat_ms = sys_time_ms;

        // 测试模式下主动解除无需无人机参与的锁存看门狗
        if (TEST_MODE != TEST_MODE_NORMAL) {
            Chassis_Unblock(DISARM_MAINLOOP_STALL);
            Chassis_Unblock(DISARM_COMM_LOST);
            Chassis_Unblock(DISARM_DRONE_STOPPED);
        }

        // 此处编写需要循环执行的代码
        Parse_Board_Uart_Data();
#if DUPLEX_SWITCH
        Board_Comm_Send_Reply();
#endif

        if (board_rx_complete_flag) {
            board_rx_complete_flag = 0;
            board_rx_ok_debug++;

            Chassis_Unblock(DISARM_COMM_LOST);
            Chassis_Unblock(DISARM_MAINLOOP_STALL);

            last_drone_rx_time_ms = sys_time_ms;
            drone_timeout_debug = 0;

            uint8_t stop_event = Board_Comm_Consume_Stop_Event();
            uint8_t drone_running = (!stop_event && uart_data[6] >= 0.5f);
            if (drone_running) {
                Chassis_Unblock(DISARM_DRONE_STOPPED);
            } else {
                Chassis_Block(DISARM_DRONE_STOPPED);
            }

            static uint32_t last_visual_time_ms = 0;
            uint32_t now = sys_time_ms;
            if (last_visual_time_ms != 0) {
                visual_loop_dt_debug = now - last_visual_time_ms;
                if (visual_loop_dt_debug > visual_loop_max_dt_debug) {
                    visual_loop_max_dt_debug = visual_loop_dt_debug;
                }
            }
            last_visual_time_ms = now;
            visual_loop_count_debug++;
            if (TEST_MODE == TEST_MODE_NORMAL && drone_running) {
                Visual_Control_Loop();   // [修复] 测试模式下禁止视觉控制与测试抢速度指令
            }
        } else {
            drone_timeout_debug = sys_time_ms - last_drone_rx_time_ms;
            // [修复] 测试模式不依赖无人机下传 (无帧亦可台架测试): 通信超时锁车仅正常模式生效
            if (TEST_MODE == TEST_MODE_NORMAL && drone_timeout_debug >= 1000U) { // 超过 1000ms 没收到通讯
                Chassis_Block(DISARM_COMM_LOST); // 置通讯丢失位 → 停车清理 (幂等)
            }
        }

        // [新增] 测试模式统一入口: 非阻塞状态机, 每轮主循环调用一次。
        #define TEST_ENTRY_WAIT_MS  3000U    // [新增] 测试模式进场等待时长
        if (TEST_MODE != TEST_MODE_NORMAL) {
            static uint32_t entry_wait_start_ms = 0U;   // 0=未开始
            static uint32_t last_wait_log_ms = 0U;
            if (entry_wait_start_ms == 0U) entry_wait_start_ms = sys_time_ms;
            
            uint32_t elapsed = (uint32_t)(sys_time_ms - entry_wait_start_ms);
            if (elapsed < TEST_ENTRY_WAIT_MS) {
                if ((uint32_t)(sys_time_ms - last_wait_log_ms) >= 500U) {
                    last_wait_log_ms = sys_time_ms;
                    wireless_uart_send_string("Waiting... (");
                    wireless_uart_send_int((int32_t)((TEST_ENTRY_WAIT_MS - elapsed) / 1000U + 1U));
                    wireless_uart_send_string("s)\r\n");
                }
            } else {
                Test_Execute();
            }
        }

        seekfree_assistant_data_analysis();

        // 2. 检查是否有参数更新 (遍历所有通道)——无线调参
        for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
            if (seekfree_assistant_parameter_update_flag[i]) {
                seekfree_assistant_parameter_update_flag[i] = 0;
                Wireless_Update(i + 1, seekfree_assistant_parameter[i]); 
                wireless_uart_send_string("Param Updated\r\n");
            }
        }

/* 无线串口打印开始 (200Hz, 完全由主循环非阻塞调度, 仅在运动时输出) */
        static uint32_t last_speed_print_ms = 0U;
        if ((uint32_t)(sys_time_ms - last_speed_print_ms) >= 5U) { // 5ms = 200Hz
            last_speed_print_ms = sys_time_ms;
            if (Test_Is_Moving()) {
                wireless_uart_output_actual_speed(); // 输出四轮合成的实际 (vx, vy)，单位 m/s
            }
        }
/* 无线串口打印结束 */

        system_delay_ms(1); // 稍微延时
        
        pid_lf.kp = KP; pid_lf.ki = KI; pid_lf.kd = KD; pid_lf.max_i = MAX_I;
        pid_rf.kp = KP; pid_rf.ki = KI; pid_rf.kd = KD; pid_rf.max_i = MAX_I;
        pid_lb.kp = KP; pid_lb.ki = KI; pid_lb.kd = KD; pid_lb.max_i = MAX_I;
        pid_rb.kp = KP; pid_rb.ki = KI; pid_rb.kd = KD; pid_rb.max_i = MAX_I;
        pid_yaw_hold.kp = YAW_KP; pid_yaw_hold.ki = YAW_KI; pid_yaw_hold.kd = YAW_KD; pid_yaw_hold.max_i = YAW_MAX_I;
        
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
void Wireless_Update(uint8_t ch, float val) {
    switch (ch) {
        case 1:
            YAW_RATE_KP = val;
            break;
        case 2:
            YAW_RATE_KI = val;
            break;
        case 3:
            YAW_RATE_KD = val;
            break;
        case 4:
            KP = val;
            break;
        case 5:
            KI = val;
            break;
        case 6:
            KD = val;
            break;
        case 7:
            feedforward_hesitate_ms = (uint32_t)val; // [新增] 无人机前馈犹豫期 (ms)
            break;
        case 8:
            if(val == 1){
              Chassis_Block(DISARM_MANUAL);   // 人工急停
            }
            if(val== 0){
                Chassis_Unblock(DISARM_MANUAL); // 解除人工急停
                seekfree_assistant_data_analysis();

                for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
                    if (seekfree_assistant_parameter_update_flag[i]) {
                        seekfree_assistant_parameter_update_flag[i] = 0;
                    }
                }
            }
            break;
        default:
            break;
    }
}
