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

void Wireless_Update(uint8_t ch, float val);
int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // 时钟配置及系统初始化<务必保留>
    
    debug_init();                       // 调试串口初始化
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(1500);
    Board_Comm_Init();

    IMU_Car_RC_Init();
    Encoder_Init();
    Mecanum_Init();
    wireless_uart_init_();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    
    pit_ms_init(PIT_CH1, 100);
    //pit_ms_init(PIT_CH2, 20); // 视觉控制周期 20ms
    
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    pit_ms_init(PIT_CH0, 1);
    while (imu_car_rc_data.is_calibrated == 0) {
        Parse_Board_Uart_Data();
        seekfree_assistant_data_analysis();
        system_delay_ms(1); 
    }
    Mecanum_Unlock();

    test_program_1();
    // 此处编写用户代码 例如外设初始化代码等
    for(;;)
    {
        // 此处编写需要循环执行的代码
        // 此处编写需要循环执行的代码
        // 此处编写需要循环执行的代码
        Parse_Board_Uart_Data();

        static uint32_t drone_timeout_cnt = 0; // [新增] 无人机通讯看门狗计数器

        // if (board_rx_complete_flag) {
        //     board_rx_complete_flag = 0;
            
        //     // 【核心修复：断连恢复】如果之前处于失联保护状态，现在重新连上了，必须恢复动力！
        //     if (drone_timeout_cnt >= 300) {
        //         EN = 1;             // 恢复 PID 输出使能
        //         Mecanum_Unlock();   // 恢复底层控制锁
        //     }
            
        //     drone_timeout_cnt = 0; // 成功收到无人机数据，喂狗清零
        //     //Visual_Control_Loop();
        // } else {
        //     drone_timeout_cnt++;
        //     // 主循环中有 system_delay_ms(1)，因此每次自增大约是 1ms 
        //     if (drone_timeout_cnt > 300) { // 超过 300ms 没收到通讯
                
        //         // 【核心修改：直接底层停车】不需要再调用视觉环了
        //         EN = 0;               // 1. 切断 PID 最终输出
        //         Mecanum_Stop();       // 2. 清空目标速度、清空 PID 积分、锁定底盘 PWM
                
        //         drone_timeout_cnt = 300; // 卡住计数器防止溢出
        //     }
        // }

        seekfree_assistant_data_analysis();

        // 2. 检查是否有参数更新 (遍历所有通道)——无线调参
        for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
            // 如果第 i 个通道有数据更新标志
            if (seekfree_assistant_parameter_update_flag[i]) {
                // 清除标志位
                seekfree_assistant_parameter_update_flag[i] = 0;
                
                // 将参数应用到 PID (通道号 = 索引 + 1)
                // seekfree_assistant_parameter[i] 是接收到的浮点数值
                Wireless_Update(i + 1, seekfree_assistant_parameter[i]); 
                
                // 可选：通过无线串口回传确认，告诉上位机收到并更新了
                wireless_uart_send_string("Param Updated\r\n");
            }
        }

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
            KP = val;
            break;
        case 2:
            KI = val;
            break;
        case 3:
            KD = val;
            break;
        case 4:
            YAW_KP = val;
            break;
        case 5:
            YAW_KI = val;
            break;
        case 6:
            YAW_KD = val;
            break;
        case 8:
            if(val == 1){
              Mecanum_Stop();
            }
            if(val== 0){
                //wireless_uart_send_string("done");
                Mecanum_Unlock();
                test_program_1();
            }
        default:
            break;
    }
}