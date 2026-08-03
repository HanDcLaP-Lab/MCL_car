#include "test.h"
#include "mecnum.h"
#include "zf_common_headfile.h"
#include <math.h>

/**
 * @brief 麦克纳姆轮测试程序0
 * 停止
 */
void test_program_0(void)
{
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    system_delay_ms(1000);
}

// [新增] 无线调参处理：TEST_MODE=1 时 main() 的 for(;;) 不会执行（test_program_1 自带 while(1)），
// 主循环里的 seekfree_assistant_data_analysis + 参数应用代码 + 末尾的全局参数写入 PID 结构体都不会跑，
// 因此复制一份到这里。无线串口接收本身由 uart2 ISR (wireless_module_uart_handler) 常驻填充缓冲，无需另行处理。
extern void Wireless_Update(uint8_t ch, float val);   // 定义在 main_cm4.c
static void Wireless_Param_Process(void)
{
    seekfree_assistant_data_analysis();   // 无线串口接收 + 解析

    // 遍历所有通道，应用上位机下发的参数更新
    for (int i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++) {
        if (seekfree_assistant_parameter_update_flag[i]) {
            seekfree_assistant_parameter_update_flag[i] = 0;
            Wireless_Update(i + 1, seekfree_assistant_parameter[i]);
            wireless_uart_send_string("Param Updated\r\n");
        }
    }

    // 将全局参数写入 PID 结构体（对应 main_cm4.c 主循环末尾的赋值部分，测试模式下不执行）
    pid_lf.kp = KP; pid_lf.ki = KI; pid_lf.kd = KD; pid_lf.max_i = MAX_I;
    pid_rf.kp = KP; pid_rf.ki = KI; pid_rf.kd = KD; pid_rf.max_i = MAX_I;
    pid_lb.kp = KP; pid_lb.ki = KI; pid_lb.kd = KD; pid_lb.max_i = MAX_I;
    pid_rb.kp = KP; pid_rb.ki = KI; pid_rb.kd = KD; pid_rb.max_i = MAX_I;
    pid_yaw_hold.kp = YAW_KP; pid_yaw_hold.ki = YAW_KI; pid_yaw_hold.kd = YAW_KD; pid_yaw_hold.max_i = YAW_MAX_I;
}

void test_program_1(void)
{
    Chassis_Unblock(DISARM_MANUAL);

    while(1){
        Wireless_Param_Process();       // 测试模式下处理无线调参
        Mecanum_Set_Velocity(0.8f, 0.0f, 0.0f);
        system_delay_ms(3000);
        // Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        // system_delay_ms(500);
        Wireless_Param_Process();  
        Mecanum_Set_Velocity(-0.8f, 0.0f, 0.0f);
        system_delay_ms(3000);
        // Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        // system_delay_ms(3000);
    }

    Chassis_Block(DISARM_MANUAL);
}

void test_program_imu(void)
{
    uint32_t print_time_ms = sys_time_ms;

    Chassis_Block(DISARM_MANUAL);
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);

    while(1){
        IMU_Car_Update_Loop();
        if ((uint32_t)(sys_time_ms - print_time_ms) >= 500U) {
            print_time_ms = sys_time_ms;
            print_imu();
        }
        system_delay_ms(1);
    }
}

/**
 * @brief 麦克纳姆轮测试程序2
 * 向右0.5m/s持续2s
 */
 void test_program_2(void)
{
    Mecanum_Set_Velocity(0.0f, 0.5f, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序3
 * 后退0.5m/s持续2s
 */
void test_program_3(void)
{
    Mecanum_Set_Velocity(-0.5f, 0.0f, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序4
 * 向左0.5m/s持续2s
 */
 void test_program_4(void)
{
    Mecanum_Set_Velocity(0.0f, -0.5f, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序5
 * 3.0rad/s角速度旋转5s
 */
void test_program_5(void)
{
    Mecanum_Set_Velocity(0.0f, 0.0f, 3.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序6
 * -3.0rad/s角速度旋转5s
 */
void test_program_6(void)
{
    Mecanum_Set_Velocity(0.0f, 0.0f, -3.0f);
    system_delay_ms(5000);
}

/**
 * @brief 麦克纳姆轮测试程序7
 * 0.5m/s速度向45°（右前方）移动2秒
 */
void test_program_7(void)
{
    float angle_rad = 45.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序8
 * 0.5m/s速度向315°（左前方）移动2秒
 */
void test_program_8(void)
{
    float angle_rad = 315.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序9
 * 0.5m/s速度向135°（右后方）移动2秒
 */
void test_program_9(void)
{
    float angle_rad = 135.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序10
 * 0.5m/s速度向225°（左后方）移动2秒
 */
 void test_program_10(void)
{ 
    float angle_rad = 225.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序11
 * 0.5m/s速度向30°（右前方）移动2秒
 */
void test_program_11(void)
{
    float angle_rad = 30.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序12
 * 0.5m/s速度向60°（右前方）移动2秒
 */
 void test_program_12(void)
{ 
    float angle_rad = 60.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序13
 * 0.5m/s速度向120°（右后方）移动2秒
 */
 void test_program_13(void)
{ 
    float angle_rad = 120.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序14
 * 0.5m/s速度向150°（右后方）移动2秒
 */
 void test_program_14(void)
{ 
    float angle_rad = 150.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序15
 * 0.5m/s速度向210°（左后方）移动2秒
 */
 void test_program_15(void)
{ 
    float angle_rad = 210.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序16
 * 0.5m/s速度向240°（左后方）移动2秒
 */
 void test_program_16(void)
{ 
    float angle_rad = 240.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序17
 * 0.5m/s速度向300°（左前方）移动2秒
 */
 void test_program_17(void)
{ 
    float angle_rad = 300.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}

/**
 * @brief 麦克纳姆轮测试程序18
 * 0.5m/s速度向330°（左前方）移动2秒
 */
 void test_program_18(void)
{ 
    float angle_rad = 330.0f * PI / 180.0f;
    float vx = 0.5f * cosf(angle_rad);
    float vy = 0.5f * sinf(angle_rad);
    Mecanum_Set_Velocity(vx, vy, 0.0f);
    system_delay_ms(2000);
}
