#ifndef _IMU_H
#define _IMU_H

#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>

// ================= 配置参数 =================
#define KP 0.8f          // 互补滤波比例增益
#define KI 0.002f        // 互补滤波积分增益
#define DT 0.001f        // 运行周期 1ms (1000Hz)
#define GRAVITY_MSS 9.8f // 标准重力加速度
// ================= Z轴融合参数 =================
// 经验值：这两个参数决定了更相信加速度计还是更相信ToF
// 如果ToF噪声大，调小这两个值；如果高度漂移大，调大这两个值
#define Z_CORRECT_POS_GAIN  0.5f   // 位置修正系数 (类似Kp)
#define Z_CORRECT_VEL_GAIN  0.05f   // 速度修正系数 (修正加速度零偏)


// 防止 PI 重复定义警告
#ifndef PI
#define PI 3.1415926535f
#endif

// ================= 核心结构体定义 =================
typedef struct {
    // --- 姿态角 (单位: 度) ---
    float roll;
    float pitch;
    float yaw;

    // --- 空间位置 (单位: cm) ---
    // 相对于上电点的位移，Z轴为 ToF 融合高度
    float x;
    float y;
    float z;

    // --- 空间速度 (单位: m/s) ---
    float vx;
    float vy;
    float vz;

    // --- 运动加速度 (单位: m/s^2) ---
    // 已去除重力分量的世界坐标系加速度
    float world_ax;
    float world_ay;
    float world_az;
    
    //初始数据
    float tof_z; ///mm
    float tof_vz; ///mm

    // --- 状态标志 ---
    uint8_t is_calibrated; // 0:校准中, 1:校准完成
} IMU_Data_t;

// ================= 全局变量声明 =================
// 在其他文件直接使用这个结构体即可获取数据
extern IMU_Data_t imu_data;

// ================= 函数声明 =================
/**
 * IMU 核心处理函数 (请在 1ms 定时器中断中调用)
 * @param tof_height_mm: ToF 测距模块的原始数据(mm)
 */
void IMU_Update_Loop(float tof_height_mm);

#endif