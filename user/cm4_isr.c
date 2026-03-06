/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一�?基于官方 SDK 接口的�??三方开源库
* Copyright (c) 2022 SEEKFREE 逐�?��?�技
*
* �?文件�? CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 �?免费�?�?
* 您可以根�?�?由软件基金会发布�? GPL（GNU General Public License，即 GNU通用�?共�?�可证）的条�?
* �? GPL 的�??3版（�? GPL3.0）或（您选择的）任何后来的版�?，重新发布和/或修改它
*
* �?开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更�?�细节�?�参�? GPL
*
* 您应该在收到�?开源库的同时收到一�? GPL 的副�?
* 如果没有，�?�参�?<https://www.gnu.org/licenses/>
*
* 额�?�注明：
* �?开源库使用 GPL3.0 开源�?�可证协�? 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
* 许可证副�?�? libraries 文件夹下 即�?�文件夹下的 LICENSE 文件
* 欢迎各位使用并传�?�?程序 但修改内容时必须保留逐�?��?�技的版权声明（即本声明�?
*
* 文件名称          cm4_isr
* �?司名�?          成都逐�?��?�技有限�?�?
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* �?改�?�录
* 日期              作�?                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            新�??12个pit周期�?�? 增加部分注释说明
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "mecnum.h"
#include "imu_car.h"

int32_t num = 0;
int32_t cnt = 0;
// **************************** PIT�?�?函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH0);
    tsl1401_collect_pit_handler(); ///逐�?�库空例程自带，意义不明
    
    cnt++;
    IMU_Car_Update_Loop();

    Mecanum_Control_Loop();
      
    
}

void pit0_ch1_isr()                     // 定时器通道 1 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH1);
    //wireless_uart_send_float(recv_pack.f_data[4]);
    // wireless_uart_send_float(recv_pack.f_data[1]);
    // wireless_uart_send_float(recv_pack.f_data[2]);
    //Current_speed_display();
    //wireless_uart_output_pid();
    
}

void pit0_ch2_isr()                     // 定时器通道 2 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH2);
    remote_uart_loop();
    Visual_Control_Loop();
}

void pit0_ch10_isr()                    // 定时器通道 10 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // 定时器通道 11 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // 定时器通道 12 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // 定时器通道 13 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // 定时器通道 14 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // 定时器通道 15 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // 定时器通道 16 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // 定时器通道 17 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // 定时器通道 18 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // 定时器通道 19 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // 定时器通道 20 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // 定时器通道 21 周期�?�?服务函数      
{
    pit_isr_flag_clear(PIT_CH21);
    
}
// **************************** PIT�?�?函数 ****************************


// **************************** 串口�?�?函数 ****************************
// 串口0默�?�作为调试串�?
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0接收�?�?
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开�? debug 串口�?�?
        debug_interrupr_handler();       // 调用 debug 串口接收处理函数 数据会�?? debug �?形缓冲区读取
#endif                                   // 如果�?改了 DEBUG_UART_INDEX 那这段代码需要放到�?�应的串口中�?�?
      
    }
    else                                 // 串口0发送中�?
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1接收�?�?
    {
        
        
      my_uart1_handler();
      
    }
    else                                // 串口1发送中�?
    {
      
        
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2接收�?�?
    {
        wireless_module_uart_handler();        // [�?复] 添加无线串口接收回调函数
      
        
        
    }
    else                                // 串口2发送中�?
    {
        
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3接收�?�?
    {
        

        
        
    }
    else                                // 串口3发送中�?
    {
      
        
        
        
    }
}

void uart4_isr (void)
{
    
    if(uart_isr_mask(UART_4))            // 串口4接收�?�?
    {
        

        
       
    }
    else                                // 串口4发送中�?
    {
      
        
        
        
    }
}

void uart5_isr (void)
{
    
    if(uart_isr_mask(UART_5))            // 串口5接收�?�?
    {
        

        
       
    }
    else                                // 串口5发送中�?
    {
      
        
        
        
    }
}

void uart6_isr (void)
{
    
    if(uart_isr_mask(UART_6))            // 串口6接收�?�?
    {
        

        
       
    }
    else                                // 串口6发送中�?
    {
      
        
        
        
    }
}
// **************************** 外部�?�?函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 �?�?服务函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 �?�?服务函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0�?口�?�部�?�?判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 �?�?服务函数     
{
    if(exti_flag_get(P02_0))
    {
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 �?�?服务函数     
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 �?�?服务函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 �?�?服务函数     
{



}

void gpio_6_exti_isr()                  // 外部 GPIO_6 �?�?服务函数     
{
	


}

void gpio_7_exti_isr()                  // 外部 GPIO_7 �?�?服务函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 �?�?服务函数     
{
    


}

void gpio_9_exti_isr()                  // 外部 GPIO_9 �?�?服务函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 �?�?服务函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 �?�?服务函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 �?�?服务函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 �?�?服务函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 �?�?服务函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 �?�?服务函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 �?�?服务函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 �?�?服务函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 �?�?服务函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 �?�?服务函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 �?�?服务函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 �?�?服务函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 �?�?服务函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 �?�?服务函数     
{



}
// **************************** 外部�?�?函数 ****************************