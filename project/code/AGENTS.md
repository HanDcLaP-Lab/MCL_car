# project/code/ — 核心应用模块

9个模块(18个文件)，每个模块一个 .c/.h 对。所有模块通过 `zf_common_headfile.h` 获取依赖。

## WHERE TO LOOK

| 模块 | 文件 | 核心职责 |
|------|------|----------|
| **mecnum** | `mecnum.c` / `mecnum.h` | 麦轮底盘主控：速度设定、运动控制循环(1ms)、视觉追踪循环(20ms)、逆运动学、PWM缩放、停车/解锁 |
| **pid** | `pid.c` / `pid.h` | PID控制器：`PID_Calculate()` (位置式) + `PID_Calculate_Incremental()` (增量式) |
| **encoder** | `encoder.c` / `encoder.h` | 4路正交编码器读取，转m/s，卡尔曼滤波平滑 |
| **imu_car_rc** | `imu_car_rc.c` / `imu_car_rc.h` | IMU660RC驱动：偏航角/累计角度/角速度，坐标系映射宏 |
| **car_board_comm** | `car_board_comm.c` / `car_board_comm.h` | UART1板间通讯：AA55协议解析，接收8个float (无人机位姿数据) |
| **car_image** | `car_image.c` / `car_image.h` | 坐标系变换：无人机相机系→车体系，输出距离+方位角 |
| **kalman_filter** | `kalman_filter.c` / `kalman_filter.h` | 一维卡尔曼滤波器 |
| **wireless_uart** | `wireless_uart.c` / `wireless_uart.h` | 无线串口调试输出 (UART2)，实时发送电机/PID/IMU/编码器数据 |
| **test** | `test.c` / `test.h` | 19个底盘运动测试程序 |

## 关键常量定义 (mecnum.h)

```c
CAR_L = 0.10f             // 前后轮轴距一半 (m)
CAR_W = 0.09f             // 左右轮距一半 (m)
WHEEL_RADIUS = 0.028f     // 轮子半径 (m)
PWM_MAX_M = 7000.0f       // PWM 最大占空比 (上限10000)
CONTROL_DT = 0.001f       // 控制周期 1ms
VISUAL_DT = 0.020f        // 视觉周期 20ms
MAX_ACCEL_X/Y/W           // 加速度限制
COAST_HOLD_MS = 200U      // 目标丢失后软滑行保持时间 (ms)
LOCK_THRESHOLD = 7        // 锁定所需连续有效跟踪帧数
JUMP_THRESHOLD_MIN = 50   // 跳变检测阈值下限 (cm)
JUMP_THRESHOLD_MAX = 200  // 跳变检测阈值上限 (cm)
JUMP_SCALE_COEF = 0.4     // 跳变阈值缩放系数 (× car_dist)
MERGE_COAST_MS = 400U     // 信标跳变滑行持续时间 (ms)
DASH_MS_MIN = 200U        // 盲冲时长下限 (ms)
DASH_MS_MAX = 500U        // 盲冲时长上限 (ms)
EDGE_DIST_CM = 200.0      // 画面边缘距离分界 (cm)
```

## 控制架构 (mecnum.c)

```
目标速度 (vx, vy, wz)
    ↓ 斜坡平滑 (加速度限制)
    ↓ 
偏航串级PID: Yaw Hold (外环,位置) → Yaw Rate (内环,速率)
    ↓
麦轮逆运动学 → 4个轮子目标速度
    ↓
4× 增量式PID (速度环,最内环)
    ↓
PWM等比例抗饱和缩放 → 4路 PWM + DIR 输出
```

## CONVENTIONS (本目录特有)

### 段落分隔符
- **模块内大段**: `// ================== 标题 ==================`
- **子段落**: `// --- 子标题 ---`
- **算法核心**: `// 【核心一】...【核心二】...`

### 改动标记
- `// [新增]` — 新加功能
- `// [修复]` — bug修复
- `// [优化]` — 性能/逻辑优化
- `// [参数调整]` — 参数变更

### include 顺序
每个 .c 文件只 `#include "zf_common_headfile.h"` + 自己的 .h。不要单独 include 子头文件。

### 全局变量声明模式
```c
// 在 .h 中:
extern PID_t pid_lf, pid_rf, pid_lb, pid_rb;
extern float KP, KI, KD, MAX_I;

// 在 .c 中:
PID_t pid_lf, pid_rf, pid_lb, pid_rb;
float KP = 3.5f, KI = 0.1f, KD = 0.05f, MAX_I = 3000.0f;
```

## ANTI-PATTERNS (本目录)

- ❌ **在头文件中定义变量** — 必须 `extern` 声明，.c 中定义
- ❌ **新增模块用 snake_case 命名函数** — 统一 `ModuleName_Action()`
- ❌ **新增 typedef 不加 `_t` 后缀** — 统一使用 `_t`
- ❌ **删除 mecnum.c 中的调试注释** — 保留 `//wireless_uart_*` 等被注释的调试代码
- ❌ **修改 PID 默认参数直接在 mecnum.c 中写死** — 参数通过无线调参动态修改
