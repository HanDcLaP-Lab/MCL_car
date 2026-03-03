#include "image.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ==========================================
// 1. 常量参数
// ==========================================
const double CX = 94.3964870095;
const double CY = 56.7202594062;
const double INV_S11 = 1.0000000000;
const double INV_S12 = 0.0000000000;
const double INV_S21 = 0.0000000000;
const double INV_S22 = 1.0000000000;
const double A0 = 68.5826350751;
const double A2 = -0.0051614074;
const double A3 = 0.0000196348;
const double A4 = -0.0000002898;

// 定义 3D 空间向量 / 2D 地面点
typedef struct { double x, y, z; } Vector3D;
typedef struct { double x, y; } Point2D;

void Image_Init(void) {
    memset(uart_data, 0, sizeof(uart_data));
}

// ==========================================
// 2. 核心算法：像素坐标 -> 3D 空间射线 (指向地面)
// ==========================================
static Vector3D pixelTo3DRay(double u, double v) {
    Vector3D ray;
    
    double u_prime = u - CX;
    double v_prime = v - CY;

    // 转换为相机物理坐标 (X向右, Y向前)
    double x = INV_S11 * u_prime + INV_S12 * v_prime;
    double y = -(INV_S21 * u_prime + INV_S22 * v_prime); // 取负，使得 Y 正向为前方

    double rho = sqrt(x * x + y * y);
    double rho2 = rho * rho;
    double z_poly = A0 + A2 * rho2 + A3 * rho2*rho + A4 * rho2*rho2;

    ray.x = x;
    ray.y = y;
    ray.z = -z_poly; 
    
    return ray;
}

// ==========================================
// 3. 无人机姿态旋转 (Pitch, Roll)
// ==========================================
static Vector3D cameraToBody(const Vector3D *cam, double pitch_deg, double roll_deg) {
    Vector3D body;
    double p = pitch_deg * M_PI / 180.0;
    double r = roll_deg * M_PI / 180.0;
    
    // 1. 应用 Pitch (绕 X 轴(右)旋转)
    double x1 = cam->x;
    double y1 = cam->y * cos(p) - cam->z * sin(p);
    double z1 = cam->y * sin(p) + cam->z * cos(p);
    
    // 2. 应用 Roll (绕 Y 轴(前)旋转)
    body.x = x1 * cos(r) + z1 * sin(r);
    body.y = y1;
    body.z = -x1 * sin(r) + z1 * cos(r);
    
    return body;
}

// ==========================================
// 4. 将射线投影到真实水平地面
// ==========================================
static Point2D projectToGround(Vector3D ray, double height) {
    Point2D ground_pt = {0.0, 0.0};
    // 如果 z >= 0，说明射线平行于地面或指向天空，无法交于地面
    if (ray.z >= 0) return ground_pt; 
    
    // 地面方程为 Z = -height，求缩放系数
    double scale = -height / ray.z;
    
    ground_pt.x = ray.x * scale;
    ground_pt.y = ray.y * scale;
    return ground_pt;
}

// ==========================================
// 5. 计算最终夹角
// ==========================================
static double calculateFinalAngle(double dx, double dy, double yaw_cam, double yaw_car) {
    // 1. 在无人机局部坐标系下，目标相对于小车的夹角
    double angle_in_drone_frame = atan2(dx, dy) * 180.0 / M_PI;
    
    // 2. 加入绝对偏航角修正
    double final_angle = angle_in_drone_frame + (yaw_cam - yaw_car);
    
    // 3. 归一化到 [0, 360) 区间
    while (final_angle < 0) final_angle += 360.0;
    while (final_angle >= 360.0) final_angle -= 360.0;
    
    return final_angle;
}

// ==========================================
// 6. 核心解算接口
// ==========================================
void Image_Solve(float car_yaw, float *dist, float *angle) {
    double u_car = (double)uart_data[0];
    double v_car = (double)uart_data[1];
    double u_target = (double)uart_data[2];
    double v_target = (double)uart_data[3];
    double roll_deg = (double)uart_data[4];
    double pitch_deg = (double)uart_data[5];
    double yaw_cam = (double)uart_data[6];
    double height = (double)uart_data[7];
    double yaw_car = (double)car_yaw;

    // 步骤1：分别获取目标和小车在相机坐标系下的 3D 射线
    Vector3D ray_target = pixelTo3DRay(u_target, v_target);
    Vector3D ray_car = pixelTo3DRay(u_car, v_car);
    
    // 步骤2：结合 Pitch 和 Roll 进行空间旋转
    Vector3D body_target = cameraToBody(&ray_target, pitch_deg, roll_deg);
    Vector3D body_car = cameraToBody(&ray_car, pitch_deg, roll_deg);
    
    // 步骤3：与地面求交点
    Point2D pt_target = projectToGround(body_target, height);
    Point2D pt_car = projectToGround(body_car, height);
    
    double dx = pt_target.x - pt_car.x;
    double dy = pt_target.y - pt_car.y;
    
    *dist = (float)sqrt(dx * dx + dy * dy);
    *angle = (float)calculateFinalAngle(dx, dy, yaw_cam, yaw_car);
}