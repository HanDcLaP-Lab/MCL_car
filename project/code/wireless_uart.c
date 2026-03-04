#include "zf_common_headfile.h"

uint8 data_buffer[32];
uint8 data_len;

void wireless_uart_init_(){
    
    if(wireless_uart_init())                                                    // 判断是否通过初始化
    {
        while(1)                                                                // 初始化失败就在这进入死循环
        {
            system_delay_ms(100);                                               // 短延时快速闪灯表示异常
        }
    }
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // 初始化正常 输出测试信息
}

void wireless_uart_get_(){
    data_len = (uint8)wireless_uart_read_buffer(data_buffer, 32);             // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
        if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
        {
            Mecanum_Stop();
            wireless_uart_send_buffer(data_buffer, data_len);                     // 将收到的消息发送回去
            memset(data_buffer, 0, 32);
            func_uint_to_str((char *)data_buffer, data_len);
            wireless_uart_send_string("\r\ndata len:");                                 // 显示实际收到的数据信息
            wireless_uart_send_buffer(data_buffer, strlen((const char *)data_buffer));    // 显示收到的数据个数
            wireless_uart_send_string(".\r\n");
        }
}
void wireless_uart_send_int(int32_t send_a)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", send_a);
    wireless_uart_send_string(buf);
}
void wireless_uart_send_float(float send_a)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", (double)send_a);
    wireless_uart_send_string(buf);
}

void wireless_uart_output_motor(void){
    wireless_uart_send_float(imu_car_data.yaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.wz);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_output.lf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_output.rf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_output.lb);
    wireless_uart_send_string(",");
    wireless_uart_send_float(motor_output.rb);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_pid(void){
    wireless_uart_send_float(target_vel.v_lf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(encoder_data.lf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.wz);
    wireless_uart_send_string(",");
    wireless_uart_send_float(pid_lf.output);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_car_data.yaw); // 增量式PID积分项无意义，改为显示Yaw角
    wireless_uart_send_string("\n");
}