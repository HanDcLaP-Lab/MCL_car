#include "test.h"
#include "mecnum.h"
#include "zf_common_headfile.h"
#include <math.h>
#include "kalman_filter.h"

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
 * 基于 EKF 估计精准横向移动2m (用于标定 ODOM_FACTOR_Y)
 */
void test_program_3(void)
{
    float distance = 0.0f;
    float target_distance = 2.0f; // 目标横向移动 2 米
    
    Mecanum_Unlock();
    
    // 初始以 0.3m/s 的速度横向起步 (vy 传正数，通常代表向左横移)
    Mecanum_Set_Velocity(0.0f, 0.3f, 0.0f);
    
    while (distance < target_distance) {
        // 积分步长: 每 10ms 采样一次速度
        system_delay_ms(10);
        
        // 获取当前 EKF 估计的车体坐标系横向线速度 (V_y_body)
        float current_vy = chassis_ekf.X_data[4];
        
        // 积分累加里程: 距离 = 速度绝对值 * 时间 (0.01秒)
        distance += fabsf(current_vy) * 0.01f;
        
        // 快到终点时 (剩余 0.2m)，主动减速到 0.1m/s 防止惯性滑出
        if (target_distance - distance < 0.2f) {
            Mecanum_Set_Velocity(0.0f, 0.1f, 0.0f);
        }
    }
    
    // 到达 2m，立即急停锁死
    Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
    system_delay_ms(1000);
    Mecanum_Stop();
}

/**
 * @brief 麦克纳姆轮测试程序4
 * 基于 EKF 全局坐标，执行多航点闭环直线移动测试 (无限循环直角三角形)
 * 路径: (0,0) -> (1,0) -> (1,-1) -> (0,0) -> 循环
 */
 void test_program_4(void)
{
    // 1. 定义目标航点数组: {X_world, Y_world}
    float waypoints[3][2] = {
        {1.0f,  0.0f},  // 第一点: 正前方 1 米
        {1.0f, -1.0f},  // 第二点: 保持 X，向右方移动 1 米
        {0.0f,  0.0f}   // 第三点: 返回原点
    };
    int num_waypoints = 3;

    Mecanum_Unlock();

    // 无限循环，方便观察累积误差
    while (1) {
        // 2. 依次遍历并驶向每一个航点
        for (int i = 0; i < num_waypoints; i++) {
            float target_x = waypoints[i][0];
            float target_y = waypoints[i][1];
            float dist = 999.0f;

            // 当距离目标点大于 5cm 时，持续控制调整
            while (dist > 0.05f) {
                system_delay_ms(10); // 控制周期 10ms (100Hz)

                // ① 获取当前 EKF 的全局坐标与航向角
                float current_x = chassis_ekf.X_data[0];
                float current_y = chassis_ekf.X_data[1];
                float current_theta = chassis_ekf.X_data[2];

                // ② 计算世界坐标系下的位置误差
                float err_x = target_x - current_x;
                float err_y = target_y - current_y;
                dist = sqrtf(err_x * err_x + err_y * err_y);

                // ③ 将世界坐标系误差旋转到车体坐标系 (极为关键！)
                // 旋转矩阵变换: [err_body] = R(-theta) * [err_world]
                float cos_theta = cosf(current_theta);
                float sin_theta = sinf(current_theta);
                
                float err_x_body = err_x * cos_theta + err_y * sin_theta;
                float err_y_body = -err_x * sin_theta + err_y * cos_theta;

                // ④ 纯比例(P)位置环控制器，直接将距离误差映射为车体线速度
                float kp = 1.2f; // 比例系数: 误差1米时提供1.2m/s的趋势速度
                float vx = kp * err_x_body;
                float vy = kp * err_y_body;

                // ⑤ 速度向量圆滑限幅 (最大不超过 0.35 m/s，保证移动平稳防滑)
                float max_v = 0.35f;
                float current_v_mag = sqrtf(vx * vx + vy * vy);
                if (current_v_mag > max_v) {
                    vx = (vx / current_v_mag) * max_v;
                    vy = (vy / current_v_mag) * max_v;
                }

                // ⑥ 下发速度指令 (自转 wz 设为0，由底层的 Yaw 串级 PID 自动锁死车头不偏转)
                Mecanum_Set_Velocity(vx, vy, 0.0f);
            }

            // 到达当前航点，精确刹车并停顿 1 秒供观察
            Mecanum_Set_Velocity(0.0f, 0.0f, 0.0f);
            system_delay_ms(1000); 
        }
    }
    
    // Mecanum_Stop(); // 此处无法到达，保留作为代码习惯
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