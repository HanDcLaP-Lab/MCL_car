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
10. pit_ms_init(PIT_CH1, 40)           // 40ms 定时器 (调试输出)
11. pit_ms_init(PIT_CH0, 1)            // 1ms 定时器 (主控制循环)
12. 等待 IMU 校准完成                   // while(is_calibrated == 0)
13. Mecanum_Unlock()                    // 解锁电机
14. 进入主循环 for(;;)
```

## ISR 分配表

| 中断 | 周期/触发 | 处理内容 |
|------|-----------|----------|
| **PIT_CH0** | 1ms | ⚡ 硬实时：`IMU_Car_RC_Update_Loop()` + `Mecanum_Control_Loop()` |
| **PIT_CH1** | 40ms | 调试数据输出（默认注释掉） |
| **PIT_CH2** | 未初始化 | 预留，原用于视觉控制周期 |
| **UART0** | RX中断 | 调试串口 `debug_interrupr_handler()` |
| **UART1** | RX中断 | 板间通讯：接收无人机数据写入 `board_rx_fifo` |
| **UART2** | RX中断 | 无线模块：`wireless_module_uart_handler()` |
| **UART3-6** | - | 空占位 |
| **GPIO_13** | EXTI | IMU660RC 数据就绪回调 `imu660rc_callback()` |
| **GPIO_0-23** | - | 大部分为空占位 |

## 主循环逻辑 (for(;;))

```
1. Parse_Board_Uart_Data()              // 解析无人机数据包
2. 无人机急停判断 (uart_data[6] < 0.5f)  // Z轴高度过低 → 停车
3. 无人机通讯看门狗:
   - 收到数据 → 喂狗清零，跑 Visual_Control_Loop()
   - 超时1000ms → EN=0, Mecanum_Stop(), 卡住计数器
   - 断连恢复 → EN=1, Mecanum_Unlock()
4. seekfree_assistant_data_analysis()   // 无线调参数据处理
5. 遍历参数更新标志 → Wireless_Update()   // 应用调参值
6. 同步 PID 参数到4个电机 + 偏航环
7. system_delay_ms(1)                   // 主循环周期 ~1ms
```

## 关键全局变量 (本目录定义)

| 变量 | 类型 | 用途 |
|------|------|------|
| `board_rx_complete_flag` | `volatile uint8_t` | 无人机数据包接收完成标志 (ISR设置) |
| `temp_rx_dat` | `uint8_t` | UART1 单字节接收缓冲 |
| `drone_timeout_cnt` | `static uint32_t` | 无人机通讯看门狗计数器 (main loop内) |

## ANTI-PATTERNS (本目录)

- ❌ **删除 `clock_init(SYSTEM_CLOCK_160M)`** — 时钟初始化必须保留
- ❌ **在 ISR 中加 `system_delay_ms()` 或长时间阻塞** — ISR 必须快速返回
- ❌ **修改 ISR 函数名** — ISR 名称由链接脚本/向量表固定，改名会导致中断不触发
- ❌ **删除空占位 ISR** — 保留它们，防止未初始化中断触发 HardFault
