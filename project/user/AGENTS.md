# project/user/ — 程序入口与中断服务

2个文件，程序启动和所有硬件中断在此处理。

## 文件说明

| 文件 | 作用 |
|------|------|
| `main_cm4.c` | `main()` 入口：初始化序列 + 主循环 (无人机通讯、视觉追踪、无线调参) |
| `cm4_isr.c` | 所有 ISR：PIT 定时器中断、UART 串口中断、GPIO 外部中断 |

## 初始化序列 (main_cm4.c)

```
1. clock_init(SYSTEM_CLOCK_160M)        // 时钟 160MHz，务必保留
2. debug_init()                         // 调试串口
3. system_delay_ms(1500)               // 等待外设上电稳定
4. Board_Comm_Init()                    // UART1 板间通讯
5. IMU_Car_RC_Init()                    // IMU660RC 初始化
6. Encoder_Init()                       // 4路编码器
7. Mecanum_Init()                       // 麦轮底盘 (GPIO/PWM/PID)
8. wireless_uart_init_()                // 无线串口
9. seekfree_assistant_interface_init()  // 无线调参协议
10. pit_ms_init(PIT_CH1, 200)           // 200ms 定时器 (主循环调试打印触发)
11. pit_ms_init(PIT_CH0, 1)            // 1ms 定时器 (主控制循环)
12. 校准完成后检查 TEST_MODE             // 为1时直接进入test_program_1，不进入正式主循环
13. 进入主循环 for(;;)                    // TEST_MODE为0时执行正式控制
```

## ISR 分配表

| 中断 | 周期/触发 | 处理内容 |
|------|-----------|----------|
| **PIT_CH0** | 1ms | ⚡ 硬实时：轮询 IMU660RC 原始 acc/gyro，更新 Kalman/Mahony 姿态 + `Mecanum_Control_Loop()` |
| **PIT_CH1** | 200ms | 置位主循环调试打印标志，避免在中断内阻塞输出 |
| **PIT_CH2** | 未初始化 | 预留，原用于视觉控制周期 |
| **UART0** | RX中断 | 调试串口 `debug_interrupr_handler()` |
| **UART1** | RX中断 | 板间通讯：接收无人机数据写入 `board_rx_fifo` |
| **UART2** | RX中断 | 无线模块：`wireless_module_uart_handler()` |
| **UART3-6** | - | 空占位 |
| **GPIO_13** | EXTI | 预留；当前 IMU 使用 PIT_CH0 主动读取原始 acc/gyro 并在软件中融合 |
| **GPIO_0-23** | - | 大部分为空占位 |

## 主循环逻辑 (for(;;))

```
1. Parse_Board_Uart_Data()              // 排空FIFO并保留最新帧，锁存批次stop事件
2. 无人机急停判断                       // 批次内任一car_en=0优先，下一批新帧才允许恢复
3. 无人机通讯看门狗:
   - 收到数据 → 记录 `last_drone_rx_time_ms`，处理stop事件或最新普通帧
   - 当前时间距最后合法帧达到1000ms → Chassis_Block(DISARM_COMM_LOST)
   - 注意：重连只清 COMM_LOST 位，人工急停(DISARM_MANUAL)保持 → 不会自动跑起来
4. seekfree_assistant_data_analysis()   // 无线调参数据处理
5. 遍历参数更新标志 → Wireless_Update()   // 应用调参值
6. 同步 PID 参数到4个电机 + 偏航环
7. system_delay_ms(1)                   // 主循环周期 ~1ms
```

## 关键全局变量 (本目录定义)

| 变量 | 类型 | 用途 |
|------|------|------|
| `board_rx_complete_flag` | `volatile uint8_t` | 本次解析至少收到一个合法无人机数据包 |
| `board_rx_stop_pending` | `volatile uint8_t` | 本批次任一合法帧出现car_en=0 |
| `board_rx_fifo_write_fail_count` | `volatile uint32_t` | UART ISR因FIFO满/忙写入失败计数 |
| `temp_rx_dat` | `uint8_t` | UART1 单字节接收缓冲 |
| `last_drone_rx_time_ms` | `uint32_t` | 最近合法无人机数据包的物理时刻 (main局部变量) |

## ANTI-PATTERNS (本目录)

- ❌ **删除 `clock_init(SYSTEM_CLOCK_160M)`** — 时钟初始化必须保留
- ❌ **在 ISR 中加 `system_delay_ms()` 或长时间阻塞** — ISR 必须快速返回
- ❌ **修改 ISR 函数名** — ISR 名称由链接脚本/向量表固定，改名会导致中断不触发
- ❌ **删除空占位 ISR** — 保留它们，防止未初始化中断触发 HardFault

## 底盘使能状态机 (6.15a 重构后)

底盘是否输出动力，由单一掩码 `disarm_flags` (`chassis_arm.c`) 决定：**四位全清 (==0) 才武装**。
每个停车来源只置/清自己那一位，互不干扰；任一位为 1 即锁定停车。接口在 `chassis_arm.h`：
`Chassis_Block(reason)` / `Chassis_Unblock(reason)` / `Chassis_Is_Armed()` / `Chassis_Get_Disarm_Flags()`。

```
   bit0 DISARM_UNCALIBRATED   置: IMU 未校准       清: IMU 校准完成        ← 1ms ISR (mecnum.c)
   bit1 DISARM_COMM_LOST      置: 看门狗>1000ms    清: 收到无人机下传数据  ← 主循环 (main_cm4.c)
   bit2 DISARM_MANUAL         置: 无线 ch8 == 1    清: 无线 ch8 == 0       ← 调参回调 (main_cm4.c)
   bit3 DISARM_DRONE_STOPPED  置: uart_data[6]<0.5 清: uart_data[6]>=0.5  ← 主循环 (main_cm4.c)

   ┌──────────────────────┐  任一 Chassis_Block(reason)   ┌──────────────────────┐
   │ ARMED  flags == 0    │ ────────────────────────────► │ DISARMED  flags != 0 │
   │ 跑斜坡+yaw+轮速PID   │  置位→Chassis_Apply_Stop():   │ 电机输出强制 0       │
   │ 驱动电机             │ ◄──────────────────────────── │ 控制环跳过电机赋值   │
   └──────────────────────┘  最后一个 Unblock 清零→复位起步└──────────────────────┘

   幂等: Block 同一位重复调用→直接返回 (杜绝 1ms 狂调清理)。
   关键安全路径: 人工急停(bit2) 期间断连(bit1) → 重连只清 bit1 → 仍 DISARMED → 必须 ch8=0 才跑。
   已知问题: disarm_flags 初始值仅设 DISARM_UNCALIBRATED，未设 DISARM_COMM_LOST，IMU 校准后到首次
            收到 drone 数据前短暂处于 ARMED（无实际风险，target_vel=0）。
```

## 代码组织 (6.15a 重构后)

`mecnum.c` 原 696 行按功能域拆为三个文件：

| 文件 | 职责 | 行数(估) |
|------|------|----------|
| `chassis_arm.c/h` | 使能状态机：disarm_flags + Block/Unblock + 停车清理 | ~70 |
| `car_image.c/h` | 视觉跟踪：State0~3_Handler + 盲冲(距离触发) + 方向/速度/距离 EMA + Image_Solve | ~530 |
| `mecnum.c/h` | 底层运动控制：电机引脚 + 逆运动学 + 轮速PID + 偏航串级 | ~304 |

依赖方向: `mecnum.c` → `chassis_arm.c` → `car_image.c`（mecnum 调用 Visual_Brake_Check；chassis_arm 调用 Visual_State_Reset）。
