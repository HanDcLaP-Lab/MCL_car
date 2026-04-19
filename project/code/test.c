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

/**
 * @brief 麦克纳姆轮测试程序1
 * 前进0.5m/s持续2s
 */
void test_program_1(void)
{
    Mecanum_Unlock();

    while(1){
        Mecanum_Set_Velocity(0.25f, 0.0f, 0.0f);
        system_delay_ms(5000);
        Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        system_delay_ms(2000);
        Mecanum_Set_Velocity(-0.25f, 0.0f, 0.0f);
        system_delay_ms(5000);
        Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
        system_delay_ms(2000);
    }
    
    Mecanum_Stop();
}

/**
 * @brief 麦克纳姆轮测试程序2
 * 基于编码器里程计精准前进2m (用于标定 RPM_TO_MPS)
 */
 void test_program_2(void)
{
    float distance = 0.0f;
    float target_distance = 2.0f; // 目标前进 2 米
    
    Mecanum_Unlock();
    
    // 初始以 0.3m/s 的速度起步前进
    Mecanum_Set_Velocity(0.3f, 0.0f, 0.0f);
    
    while (distance < target_distance) {
        // 积分步长: 每 10ms 采样一次速度
        system_delay_ms(10);
        
        // 获取当前底盘前向线速度 (四个轮子线速度的平均值)
        float current_vx = (encoder_data.lf + encoder_data.rf + encoder_data.lb + encoder_data.rb) / 4.0f;
        
        // 积分累加里程: 距离 = 速度 * 时间 (0.01秒)
        distance += current_vx * 0.01f;
        
        // 快到终点时 (剩余 0.2m)，主动减速到 0.1m/s 防止惯性滑出
        if (target_distance - distance < 0.2f) {
            Mecanum_Set_Velocity(0.1f, 0.0f, 0.0f);
        }
    }
    
    // 到达 2m，立即急停锁死
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    system_delay_ms(1000);
    Mecanum_Stop();
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