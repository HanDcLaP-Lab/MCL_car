# project/code/ — 核心应用模块

9个模块(18个文件)，每个模块一个 .c/.h 对。所有模块通过 `zf_common_headfile.h` 获取依赖。

## WHERE TO LOOK

| 模块 | 文件 | 核心职责 |
|------|------|----------|
| **mecnum** | `mecnum.c` / `mecnum.h` | 麦轮底盘主控：速度设定、运动控制循环(1ms)、逆运动学、PWM缩放 |
| **pid** | `pid.c` / `pid.h` | PID控制器：`PID_Calculate()` (位置式) + `PID_Calculate_Incremental()` (增量式) |
| **encoder** | `encoder.c` / `encoder.h` | 4路正交编码器读取，转m/s，卡尔曼滤波平滑 |
| **imu_car_rc** | `imu_car_rc.c` / `imu_car_rc.h` | IMU660RC原始acc/gyro轮询 + Kalman/Mahony姿态融合：roll/pitch、偏航角/累计角度/角速度，坐标系映射宏 |
| **car_board_comm** | `car_board_comm.c` / `car_board_comm.h` | UART1板间通讯：AA55协议解析、最新帧更新与批次急停锁存 |
| **car_image** | `car_image.c` / `car_image.h` | 视觉追踪状态机：坐标系变换、双目标跟踪记忆、盲冲/滑行控制 |
| **kalman_filter** | `kalman_filter.c` / `kalman_filter.h` | 一维卡尔曼滤波器 |
| **wireless_uart** | `wireless_uart.c` / `wireless_uart.h` | 无线串口调试输出 (UART2)，实时发送电机/PID/IMU/编码器数据 |
| **test** | `test.c` / `test.h` | 19个底盘运动测试程序 |

## 关键常量定义 (mecnum.h)

```c
CAR_L = 0.10f             // 前后轮轴距一半 (m)
CAR_W = 0.09f             // 左右轮距一半 (m)
WHEEL_RADIUS = 0.028f     // 轮子半径 (m)
PWM_MAX_M = 5000.0f       // PWM 最大占空比 (上限10000)
CONTROL_DT = 0.001f       // 控制周期 1ms
VISUAL_DT = 0.020f        // 旧视觉周期常量；当前 Visual_Control_Loop 由收包触发
MAX_ACCEL_X/Y/W           // 加速度限制
CAR_VALID_MS = 50U         // 小车坐标保质期 (ms)
TARGET_VALID_MS = 50U      // 目标坐标保质期 (ms)
ANGLE_VALID_MS = 600U      // 合成角度保质期 (ms)，替代旧 COAST_HOLD_MS
ANGLE_MATCH_COS = 0.964f   // cos(15.5°)，同目标角度匹配阈值
DASH_DIST_CM = 50.0f       // 车-目标距离低于此值时触发盲冲 (cm)
DASH_MS_MIN = 200U        // 盲冲时长下限 (ms)
DASH_MS_MAX = 700U        // 盲冲时长上限 (ms)
DASH_EXTRA_MS = 100       // 盲冲在计算时长基础上固定增加 100ms
DASH_SPEED_MIN_MPS = 0.20f // 可信接近速度下限, 兼可靠性门下界
DASH_SPEED_MAX_MPS = 1.20f // 可信接近速度上限
Target identity filter = 5 个 Expiring_Slot_t (car/target/latest/adopted/pending) 替代两套 coast；
  角度差 < 15.5° 时采纳新观测，否则暂存 pending 直到旧目标过期
Dash 方向 = adopted_angle 方向向量 EMA (α=0.3, Cartesian 坐标系, 360° 环绕安全)
Dash 速度 = 帧间距离差 EMA (α=0.3) + 可靠性门 (≥4 样本且 > DASH_SPEED_MIN_MPS → 用估计值; 否则 TARGET_SPEED)
Dash 距离 = 距离 EMA (α=0.3, 防末帧噪声)
Dash 触发 = state3 期间车-目标距离 < DASH_DIST_CM (50cm) 时在 State3_Handler 内直接触发
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
