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
    wireless_uart_send_float(imu_car_rc_data.yaw); // 增量式PID积分项无意义，改为显示Yaw角
    wireless_uart_send_string("\n");
}

void wireless_uart_output_target(void){
    wireless_uart_send_float(uart_data[0]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[1]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[2]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[3]);
    wireless_uart_send_string(",");
    //wireless_uart_send_float(ang_out);
    //wireless_uart_send_string(",");
    wireless_uart_send_int((int32_t)Chassis_Get_Disarm_Flags());
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_car_rc_data.yaw_total);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.v_lf);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_imu(void){
    wireless_uart_send_float(imu_car_rc_data.pitch);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_car_rc_data.roll);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_car_rc_data.yaw);
    wireless_uart_send_string(",");
    wireless_uart_send_float(imu_car_rc_data.yaw_total);
    // wireless_uart_send_int(imu660rc_acc_x);
    // wireless_uart_send_string(",");
    // wireless_uart_send_int(imu660rc_acc_y);
    // wireless_uart_send_string(",");
    // wireless_uart_send_int(imu660rc_acc_z);
    wireless_uart_send_string("\n");
}
void wireless_uart_output_encoder(void){
    wireless_uart_send_float(encoder_data.lf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.v_lf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(encoder_data.rf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.v_rf);
    wireless_uart_send_string(",");
    wireless_uart_send_float(encoder_data.lb);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.v_lb);
    wireless_uart_send_string(",");
    wireless_uart_send_float(encoder_data.rb);
    wireless_uart_send_string(",");
    wireless_uart_send_float(target_vel.v_rb);
    wireless_uart_send_string("\n");
}

void wireless_uart_output_coast(void) {
    extern volatile uint32_t sys_time_ms;

    uint32_t now = sys_time_ms;

    wireless_uart_send_int(uart_data[5]);
    
    if (dash_end_time > 0 && now < dash_end_time) {
        if (dash_source == 2) {
            wireless_uart_send_string(",4,");
        } else {
            wireless_uart_send_string(",3,");
        }
        wireless_uart_send_int((int32_t)(dash_end_time - now));
        wireless_uart_send_string("\n");
    }else if (visual_coast_end_time > 0 && now < visual_coast_end_time) {
        wireless_uart_send_string(",1,");
        wireless_uart_send_int((int32_t)(visual_coast_end_time - now));
        wireless_uart_send_string("\n");
    }else if (merge_coast_end_time > 0 && now < merge_coast_end_time) {
        wireless_uart_send_string(",2,");
        wireless_uart_send_int((int32_t)(merge_coast_end_time - now));
        wireless_uart_send_string("\n");
    }else{
        wireless_uart_send_string(",0,0\n");
    }
}

void wireless_uart_output_stop_debug(void) {
    extern volatile uint32_t drone_timeout_debug;
    extern volatile uint32_t board_rx_ok_debug;

    uint8_t flags = Chassis_Get_Disarm_Flags();

    wireless_uart_send_string("STOPDBG,f:");
    wireless_uart_send_int((int32_t)flags);
    wireless_uart_send_string(",arm:");
    wireless_uart_send_int((int32_t)Chassis_Is_Armed());
    wireless_uart_send_string(",ce:");
    wireless_uart_send_float(uart_data[6]);
    wireless_uart_send_string(",ls:");
    wireless_uart_send_int((int32_t)((uint8_t)uart_data[5]));
    wireless_uart_send_string(",cal:");
    wireless_uart_send_int((int32_t)imu_car_rc_data.is_calibrated);
    wireless_uart_send_string(",to:");
    wireless_uart_send_int((int32_t)drone_timeout_debug);
    wireless_uart_send_string(",rx:");
    wireless_uart_send_int((int32_t)board_rx_ok_debug);
    wireless_uart_send_string("\r\n");
}

void wireless_uart_output_comm_debug(void) {
    extern volatile uint32_t drone_timeout_debug;
    extern volatile uint32_t visual_loop_count_debug;
    extern volatile uint32_t visual_loop_dt_debug;
    extern volatile uint32_t visual_loop_max_dt_debug;

    wireless_uart_send_string("COMMDBG,rx:");
    wireless_uart_send_int((int32_t)board_rx_ok_count);
    wireless_uart_send_string(",pdt:");
    wireless_uart_send_int((int32_t)board_rx_last_dt_ms);
    wireless_uart_send_string(",pmax:");
    wireless_uart_send_int((int32_t)board_rx_max_dt_ms);
    wireless_uart_send_string(",v:");
    wireless_uart_send_int((int32_t)visual_loop_count_debug);
    wireless_uart_send_string(",vdt:");
    wireless_uart_send_int((int32_t)visual_loop_dt_debug);
    wireless_uart_send_string(",vmax:");
    wireless_uart_send_int((int32_t)visual_loop_max_dt_debug);
    wireless_uart_send_string(",to:");
    wireless_uart_send_int((int32_t)drone_timeout_debug);
    wireless_uart_send_string(",fifo:");
    wireless_uart_send_int((int32_t)fifo_used(&board_rx_fifo));
    wireless_uart_send_string("/");
    wireless_uart_send_int((int32_t)board_rx_fifo_max_used);
    wireless_uart_send_string(",ck:");
    wireless_uart_send_int((int32_t)board_rx_checksum_fail_count);
    wireless_uart_send_string(",tail:");
    wireless_uart_send_int((int32_t)board_rx_tail_fail_count);
    wireless_uart_send_string(",inv:");
    wireless_uart_send_int((int32_t)board_rx_invalid_count);
    wireless_uart_send_string(",ce:");
    wireless_uart_send_float(uart_data[6]);
    wireless_uart_send_string(",ls:");
    wireless_uart_send_int((int32_t)((uint8_t)uart_data[5]));
    wireless_uart_send_string("\r\n");
}

void wireless_uart_output_commu(void){

    wireless_uart_send_float(uart_data[0]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[1]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[2]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[3]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[4]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[5]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[6]);
    wireless_uart_send_string(",");
    wireless_uart_send_float(uart_data[7]);
    wireless_uart_send_string("\n");
}

void print_imu(void){
    printf("%f,%f,%f,%f\n",imu_car_rc_data.pitch , imu_car_rc_data.roll , imu_car_rc_data.yaw , imu_car_rc_data.yaw_total);
}
