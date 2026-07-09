# 脉轮小车 (Mecanum Wheel Car) — Project Knowledge Base

**MCU:** Infineon CYT2BL3 (Cortex-M4 @ 160MHz) | **IDE:** IAR EWARM 9.40.1 | **SDK:** SeekFree/逐飞科技 CYT2BL3 开源库

## OVERVIEW

Drone-guided mecanum-wheel autonomous chase car. An overhead drone detects the car + target via camera, sends ground coordinates over wireless UART. The car computes relative position, then drives 4 mecanum wheels (omnidirectional) to chase the target while maintaining heading via cascade yaw PID.

## CODEX WORKFLOW

- Codex MUST perform all repository changes on the `codex_branch` branch. If the current branch is different, switch to `codex_branch` before editing; create it from the current work base if it does not exist.
- Before every code or documentation edit, Codex MUST check whether other local or remote branches have newer commits, preferably with `git fetch --all --prune`, `git branch --all --verbose --no-abbrev`, and a short `git log --oneline --decorate --graph --all`.
- Codex SHOULD pull, merge, cherry-pick, or otherwise bring in relevant newer commits before editing when they affect the files or behavior being changed. Codex MUST NOT merge unrelated branch history just because it is newer.
- Codex MUST keep user or other-agent changes intact. Do not reset, checkout, or discard existing work unless the user explicitly asks for that operation.
- During reviews, Codex SHOULD mention stale or mismatched comments after parameter tuning, but MUST NOT treat them as bugs that need code fixes by default.
- Continuous or repeated parameter tuning is expected in this project. Codex SHOULD mention it when relevant, but MUST NOT classify ongoing tuning itself as a defect or try to "fix" it unless the user asks.

## STRUCTURE

```
MCL_car/
├── project/
│   ├── code/          # 核心应用模块 (9个模块，18个文件) — 你写代码的地方
│   ├── user/          # 入口 + 中断服务函数 — 初始化与ISR
│   └── iar/           # IAR IDE 工程配置 — 跳过，不要改
├── libraries/         # 第三方库 (zf_*) — 只读，不要修改
│   ├── zf_common/     #   公共头文件、时钟、调试、FIFO、中断
│   ├── zf_driver/     #   硬件驱动层 (ADC/DMA/GPIO/PIT/PWM/UART/SPI/编码器)
│   ├── zf_device/     #   外设驱动 (IMU660RC/摄像头/OLED/无线模块等)
│   ├── zf_components/ #   组件层 (seekfree_assistant 无线调参)
│   ├── sdk/           #   Infineon TRAVEO T2G 官方 SDK (Cypress HAL)
│   └── doc/           #   GPL3 许可证、版本信息
├── README.md          # 更新日志 + 引脚定义
└── .gitignore         # 忽略 project/iar
```

## WHERE TO LOOK

| 任务 | 位置 | 说明 |
|------|------|------|
| 底盘运动控制 / 麦轮解算 | `project/code/mecnum.c` | 1ms控制环、逆运动学、轮速PID |
| PID 控制器 | `project/code/pid.c` | 增量式 + 位置式 PID |
| 编码器读取 (4个电机) | `project/code/encoder.c` | 正交编码器 + 卡尔曼滤波 |
| IMU / 陀螺仪 | `project/code/imu_car_rc.c` | IMU660RC，Kalman/Mahony 姿态融合和坐标系映射 |
| 板间通讯 (接收无人机数据) | `project/code/car_board_comm.c` | UART1, 115200, AA55协议 |
| 视觉追踪状态机 / 坐标变换 | `project/code/car_image.c` | Image_Solve()、State0~4、盲冲/滑行 |
| 无线串口调试 / 调参 | `project/code/wireless_uart.c` | UART2, SEEKFREE无线模块 |
| 卡尔曼滤波器 | `project/code/kalman_filter.c` | 一维，用于编码器平滑 |
| 测试程序 | `project/code/test.c` | 19个底盘运动测试 |
| 程序入口 main() | `project/user/main_cm4.c` | 初始化序列 + 主循环 |
| 中断服务函数 | `project/user/cm4_isr.c` | PIT/UART/GPIO 全部ISR |
| 全局头文件入口 | `libraries/zf_common/zf_common_headfile.h` | 所有模块通过它 include |

## ARCHITECTURE

```
[无人机摄像头] → 无线 UART → [小车 UART1 接收]
                                    ↓
main() 主循环 (~1ms):              car_board_comm 解析8个float
  ├─ 解析无人机数据包               ├─ car_ground_pos (小车坐标)
  ├─ 无人机通讯看门狗 (1000ms超时停车) ├─ target_ground_pos (目标坐标)
  ├─ 收到新视觉包后调用 Visual_Control_Loop()
  │                                    ├─ locked_state (目标锁定状态)
  ├─ 无线调参处理                     └─ imu_data (roll/pitch/yaw/z)
  └─ 同步PID参数
                                    
PIT_CH0 ISR (1ms 硬实时):          car_image → Image_Solve()
  ├─ IMU_Car_RC_Update_Loop()        坐标变换: 无人机系→车体系
  └─ Mecanum_Control_Loop()          输出: 距离 + 方位角
       ├─ 编码器读取 + 卡尔曼
       ├─ 目标速度斜坡平滑
       ├─ 偏航角串级PID
       │   └─ 外环(角度) → 内环(角速度)
       ├─ 麦轮逆运动学
       ├─ 4× 增量式PID (轮速)
       ├─ PWM等比例抗饱和
       └─ Motor_Set_Output() → 4路PWM+DIR

安全机制:
  - 无人机断连 → 1000ms看门狗 → 强制停车
  - 目标丢失 → 最近1000ms内有可靠双目标跟踪才允许盲冲，否则滑行(coast)后停车
  - IMU未校准 → 主循环阻塞，不解锁电机
```

## CONVENTIONS

### 命名规则
- **函数**: `ModuleName_Action()` — 模块前缀 + 下划线 + PascalCase
  - `PID_Init()`, `Mecanum_Control_Loop()`, `Encoder_GetCount()`
  - 例外: `wireless_uart_*()` 系列继承逐飞库的 snake_case
- **宏**: `MODULE_PREFIX_DETAIL` — 全大写 + 下划线
  - `MOTOR_LF_PWM`, `CAR_L`, `MAX_ACCEL_X`, `BOARD_BAUDRATE`
- **类型**: `XXX_t` 后缀 (主流风格)
  - `PID_t`, `Target_t`, `IMU_Car_RC_Data_t`
  - 例外: `Encoder`, `KalmanFilter1` (无后缀)
- **变量**: snake_case — `encoder_data`, `imu_car_rc_data`, `target_vel`
- **全局常量 (可调参)**: 全大写 — `KP`, `KI`, `KD`, `YAW_KP`, `MAX_I`

### Include Guard
```c
#ifndef _MODULENAME_H
#define _MODULENAME_H
// ...
#endif
```
例外: `test.h` 使用 `TEST_H_`

### 注释风格
- **段落分隔**: `// ================== 标题 ==================` (code/ 目录)
- **大段分隔**: `// **************************** 标题 ****************************` (user/ 目录)
- **子段落**: `// --- 子标题 ---`
- **函数文档**: `/** @brief ... @param ... @return ... */` (Doxygen 风格)
- **改动标记**: `// [新增]`, `// [修复]`, `// [优化]` 标记代码改动
- **核心逻辑**: `// 【核心一】...【核心二】...` 标记算法关键步骤
- 注释使用中文为主，可以中英混合

### 错误处理
- **初始化失败**: `while(1) { system_delay_ms(100); }` — 死循环等待
- **通讯校验**: AA55头 + checksum 状态机 (car_board_comm.c)
- **通讯超时**: 计数看门狗 → `Chassis_Block(DISARM_COMM_LOST);`
- **PID死区**: 误差 < 0.001 时清零积分并返回0
- **电机死区**: 目标速度 < 0.01m/s 且误差 < 0.03 时输出0

### 调试打印
- **有线打印**: 使用 `printf()`。
- **无线打印**: 使用 `wireless_uart_*()` / `wireless_uart` 开头的函数。
- **数值类格式**: 数值之间用 `,` 分隔，结尾使用 `\r\n`，例如 `"dat1,dat2,dat3\r\n"`。

### 坐标系 (来自 README.md)
```
vx > 0 → 车头前进方向    vx < 0 → 后退
vy > 0 → 向右            vy < 0 → 向左
wz > 0 → 俯视逆时针(左转) wz < 0 → 相反
yaw角: 顺时针为正
```

## ANTI-PATTERNS (本项目内禁止)

- ❌ **修改 `libraries/` 下的任何文件** — 第三方只读库
- ❌ **修改 `project/iar/` 下的工程配置** — 除非你确定知道在做什么
- ❌ **使用 `const` 修饰函数参数** — 项目未建立此惯例
- ❌ **使用 `assert()`, `goto`, `inline`, `__attribute__`** — 项目不使用这些
- ❌ **引入新的类型后缀风格** — 新增 typedef 统一用 `_t` 后缀
- ❌ **在头文件中定义可变全局变量** — 用 `extern` 声明，.c 中定义
- ❌ **删除被注释掉的调试代码** — 它们可能在后续调试中有用
- ❌ **删除未使用的声明** — 声明但未用过的变量/结构体/函数不删除，后续可能会使用

## NOTES

- **主头文件**: 所有 .c 通过 `#include "zf_common_headfile.h"` 引入一切，不要单独 include 子头文件
- **编译环境**: 需要在 IAR 9.40.1 中打开 `project/iar/cyt2bl3.eww`，选 Debug_m4 配置编译
- **编码格式**: main 和 isr 文件使用 UTF-8
- **无线调参**: 通过 SEEKFREE Assistant 上位机 + 无线串口模块实时调整 PID 参数
- **IMU 校准**: 上电后需等待 `imu_car_rc_data.is_calibrated == 1`，期间电机锁定
- **急停**: 无线调参通道8 设为1 → 停车；设为0 → 解锁
- **版本号格式**: README.md 使用 `X.Ya/b/c` 格式记录更新
