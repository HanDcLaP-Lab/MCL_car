# TOFIX

Known hazards collected from Codex reviews through 2026-06-09.

Scope: `project/code`, `project/user`, and only the library behavior needed to explain project risks. This file is a working checklist, not a patch plan. Per project memory, stale comments after tuning and continuous parameter tuning should be mentioned but not treated as bugs by default.

## P1 - Fix Or Verify Before More Field Runs

### 1. [FIXED] Quadrature encoder count may be reset twice

- Evidence:
  - `project/code/encoder.c:18-32` calls `encoder_get_count(...)`, then `encoder_clear_count(...)` for all four wheels.
  - `libraries/zf_driver/zf_driver_encoder.c:146-147` already resets quadrature mode counters to `0x00008000` inside `encoder_get_count(...)`.
  - `libraries/zf_driver/zf_driver_encoder.c:167` resets quadrature counters to `0` in `encoder_clear_count(...)`.
- Risk:
  - In quadrature mode, the next stationary read after `encoder_clear_count()` can be interpreted relative to `0x00008000`, producing a huge false speed. This can directly disturb the 1ms wheel-speed PID, braking, and launch behavior.
- Suggested direction:
  - For quadrature mode, do not call `encoder_clear_count()` after `encoder_get_count()`, or make the clear function reset to the same midpoint used by the driver.

### 2. UART1 TX and right-rear PWM both use P06_1

- Evidence:
  - `project/code/car_board_comm.h:9` uses `UART1_TX_P06_1`.
  - `project/code/mecnum.h:32` uses `TCPWM_CH00_P06_1` for `MOTOR_RB_PWM`.
  - `project/code/car_board_comm.c:32-33` initializes UART1 and also initializes `MOTOR_RB_PWM`.
  - `project/code/mecnum.c:118` initializes `MOTOR_RB_PWM` again.
  - Driver mapping confirms `UART1_TX_P06_1 -> P6_1_SCB4_UART_TX` and `TCPWM_CH00_P06_1 -> P6_1_TCPWM0_LINE0`.
- Risk:
  - The later pin mux init can override the earlier one. Either board communication TX or right-rear PWM may be unreliable.
- Suggested direction:
  - Move either UART TX or right-rear PWM to a non-conflicting pin. Remove the temporary `pwm_init(MOTOR_RB_PWM, ...)` from board comm once the hardware mapping is settled.

### 3. [FIXED] Stop/unlock does not clear blind-dash and visual static state

- Evidence:
  - `project/code/mecnum.c:136-159` `Mecanum_Stop()` clears speed, PWM, PID, and lock state.
  - `project/code/mecnum.c:161-180` `Mecanum_Unlock()` clears speed, PWM, and PID.
  - Neither clears `dash_end_time`, `rush_cooldown_end_time`, `rush_sign`, nor old visual static state such as last velocity, lock memory, coast timer, and merge coast timer.
  - `project/code/mecnum.c:214-229` blind dash is processed before new visual state.
- Risk:
  - If emergency stop or timeout happens during blind dash, a quick unlock can resume the old dash velocity until `dash_end_time` expires.
- Suggested direction:
  - Add a central visual-control state reset path and call it from stop/unlock and timeout recovery.

## P2 - Behavior And Robustness Risks

### 4. [FIXED] Hardware dash timeout only zeros target velocity

- Evidence:
  - `project/code/mecnum.c:403-406` clears `target_vel.vx/vy/wz` when `dash_end_time` expires.
  - It does not clear `dash_end_time`, `rush_sign`, cooldown, or visual `last_vx/last_vy`.
- Risk:
  - The 1ms loop stops the car, but state cleanup is delayed until the next `Visual_Control_Loop()` call. If packets stop or unlock happens in the gap, state can remain stale.
- Suggested direction:
  - Let the 1ms timeout path call the same dash-finish/reset helper as the visual loop, or at least clear the global dash flags atomically.

### 5. `COAST_HOLD_MS = 100` is not a hard 100ms physical stop

- Evidence:
  - `project/code/mecnum.c:357-366` checks center-loss coast only in `Visual_Control_Loop()`.
  - User measurement says `Visual_Control_Loop()` cycle is about 50-60ms.
  - `project/user/main_cm4.c:118-124` communication timeout stops only after about 1000ms without packets.
- Risk:
  - Normal packets can extend the hold by up to one visual loop period. If packets stop after state 1/2 sets the hold velocity, the 1ms loop has no `coast_end_time` guard and the car can keep the last command until the 1000ms watchdog.
- Suggested direction:
  - Promote center-loss coast state to a global/visible timer that the 1ms control loop can enforce, similar to `dash_end_time`.

### 6. Main loop and 1ms ISR share control state without protection

- Evidence:
  - `target_vel` is defined at `project/code/mecnum.c:26`.
  - Main/visual code writes `target_vel.vx/vy/wz` through `Mecanum_Set_Velocity()`.
  - `Mecanum_Control_Loop()` reads and also writes `target_vel` at `project/code/mecnum.c:403-406`.
  - PID parameters are updated in `project/user/main_cm4.c:148-152` while the 1ms ISR uses them.
- Risk:
  - Multi-field float structures can be read half-updated, and the compiler is not told these values cross ISR/main contexts. This can show up as rare one-cycle velocity/PID spikes.
- Suggested direction:
  - Use a snapshot/command-buffer pattern or short critical sections for multi-field updates. Mark simple shared flags/counters volatile where appropriate.

### 7. Yaw hold target appears to be global 0 degrees, not current heading

- Evidence:
  - `project/code/imu_car_rc.c:55-56` initializes `yaw_total` to the first `new_yaw`.
  - `project/code/mecnum.c:452` computes `yaw_error = 0.0f - imu_car_rc_data.yaw_total`.
- Risk:
  - If the car starts at a nonzero yaw and the intent is "hold current heading", the yaw loop will rotate toward global 0 instead. During beacon chase this can bend the path or fight the visual velocity direction.
- Suggested direction:
  - Confirm intent. If heading hold should preserve startup/current heading, store a yaw reference on unlock or mode entry and use `yaw_ref - yaw_total`.

### 8. Wireless tuning updates yaw-rate variables but not `pid_yaw_rate`

- Evidence:
  - `project/user/main_cm4.c:161-168` updates `YAW_RATE_KP`, `YAW_RATE_KI`, and `YAW_RATE_KD`.
  - `project/user/main_cm4.c:152` syncs only `pid_yaw_hold`, not `pid_yaw_rate`.
  - `project/code/mecnum.c:464` and `project/code/mecnum.c:470` use `pid_yaw_rate`.
- Risk:
  - Tuning channels 1-3 may appear to change values, but the active inner yaw-rate PID continues using initialization-time parameters.
- Suggested direction:
  - Sync `pid_yaw_rate.kp/ki/kd/max_i/out_max` in the main loop, or apply tuning through a dedicated PID parameter update helper.

### 9. IMU driver globals are updated in one interrupt context and read in another

- Evidence:
  - `libraries/zf_device/zf_device_imu660rc.c:79-82` defines IMU globals without `volatile`.
  - `libraries/zf_device/zf_device_imu660rc.c:392-394` updates quaternion/euler data from the GPIO interrupt callback.
  - `project/code/imu_car_rc.c:18-68` reads those globals in the 1ms PIT path.
- Risk:
  - `imu_car_rc_data` can be derived from a partially updated IMU sample. This is usually rare, but heading control is sensitive to yaw/yaw-rate consistency.
- Suggested direction:
  - Snapshot IMU data with interrupts briefly masked, or use a sequence counter/double buffer from the IMU callback.

### 10. [FIXED] `is_edge` can be stale before or after state transitions

- Evidence:
  - `project/code/mecnum.c:201` initializes `is_edge` to `0`.
  - `project/code/mecnum.c:290` updates it only in locked state 3.
  - State 4 and states 1/2 branch on `is_edge` at `project/code/mecnum.c:296` and `project/code/mecnum.c:343`.
- Risk:
  - If the first useful frame is state 4/1/2, or after a long non-state-3 interval, edge/center classification can use old information and choose the wrong coast/dash behavior.
- Suggested direction:
  - Derive edge/center from current packet fields where possible, or invalidate `is_edge` when leaving state 3 for long enough.

## P3 - Input Validation, Diagnostics, And Lower-Priority Issues

### 11. [FIXED] `uart_data` is trusted after checksum only

- Evidence:
  - `project/code/car_board_comm.c:89-92` copies all 8 floats directly into `uart_data`.
  - `project/code/mecnum.c:233`, `248-290`, and `301-313` use state, coordinates, yaw, and distance directly.
- Risk:
  - A checksum-valid but semantically invalid packet can feed NaN/Inf, negative distance, impossible state, or extreme coordinates into trig, sqrt, and dash duration logic.
- Suggested direction:
  - Validate finite floats, state range, `car_en`, distance range, and coordinate magnitude before setting `board_rx_complete_flag`.

### 12. SeekFree assistant parameter channel is not bounds checked

- Evidence:
  - `libraries/zf_components/seekfree_assistant.c:430-431` writes `seekfree_assistant_parameter[receive_packet->channel - 1]` without checking channel 1..8.
  - `project/user/main_cm4.c:131-139` consumes those flags.
- Risk:
  - A checksum-valid corrupted channel can write out of bounds.
- Suggested direction:
  - Prefer validating the channel before using the library output in project code, unless modifying the library is acceptable.

### 13. IMU `pitch` and calibration semantics are weak

- Evidence:
  - `project/code/imu_car_rc.c:24-25` assigns `pitch` only when `imu660rc_roll` is outside +/-90 degrees.
  - `project/code/imu_car_rc.c:68` sets `is_calibrated = 1` after the first update.
  - `project/user/main_cm4.c:71-75` waits only for that flag.
- Risk:
  - `pitch` can be stale in debug output. `is_calibrated` means "first IMU update received", not "sensor calibrated and stable".
- Suggested direction:
  - Rename/clarify the flag or add a real stabilization/calibration window if startup heading stability matters.

### 14. Direction conventions need one authoritative source

- Evidence:
  - `project/code/mecnum.h:99-100` documents `vy` as left-positive/right-negative and `wz` as CCW-positive.
  - `project/code/car_image.c:28-41` comments say angle 90 degrees is car-right and produces positive `target_speed_y`.
  - README/AGENTS notes also mention direction conventions that have changed over time.
- Risk:
  - If docs and real wheel mapping disagree, later fixes can accidentally mirror lateral motion or yaw compensation.
- Suggested direction:
  - Do not "fix" this from comments alone. Confirm with a simple command test: positive `vx`, positive `vy`, positive `wz`, then document the actual convention in one place.

### 15. Edge-loss coast currently holds speed without decay

- Evidence:
  - `project/code/mecnum.h:45` sets `COAST_DECAY` to `1.0f`.
  - `project/code/mecnum.c:329-331` and `348-350` multiply `last_vx/last_vy` by `COAST_DECAY`.
- Risk:
  - Edge-loss fallback holds full last speed for the allowed frames. This may be intentional tuning, but it increases overshoot if edge-loss classification is wrong.
- Suggested direction:
  - Mention during tuning. Do not treat as a bug unless field behavior shows this branch is causing overshoot.

### 16. Acceleration limits are effectively very loose

- Evidence:
  - `project/code/mecnum.h:21-23` sets `MAX_ACCEL_X/Y = 100.0f` and `MAX_ACCEL_W = 10.0f`.
  - `project/code/mecnum.c:414-431` applies those limits every 1ms.
- Risk:
  - For a 0.6m/s command, X/Y ramp completes in about 6ms, so smoothing is almost immediate. This is tuning, not automatically a defect.
- Suggested direction:
  - Mention during tuning and slip/overshoot analysis.

### 17. `wireless_uart_get_()` would stop the car on any received wireless data

- Evidence:
  - `project/code/wireless_uart.c:20-31` calls `Mecanum_Stop()` whenever any data is received.
  - Current main path does not call this function.
- Risk:
  - If later re-enabled for diagnostics, ordinary wireless traffic can stop the car unexpectedly.
- Suggested direction:
  - Keep unused or clearly mark as test-only. If used, parse explicit commands instead of stopping on any data.

### 18. Stale comments and tuning notes

- Examples:
  - `project/user/main_cm4.c:118` comment says 300ms while code uses 1000ms.
  - Some dash-duration comments mention older margins or frame-based behavior.
  - PID comments do not always match current tuned values.
- Risk:
  - Can mislead future debugging, but per project preference this is not a code bug by itself.
- Suggested direction:
  - Clean comments only when touching the nearby behavior for another reason.

### 19. 旋转逻辑已禁用,但 was_aligning 冻结小车的副作用还活着

- Evidence:
  - `image_ctrl.c` 中对准逻辑的核心行被改成了 `flight_target.target_yaw = 0;`，`search_yaw_seq[8]` 全为 0 (过时)。即 "暂停使用旋转逻辑"。
  - 但 `was_aligning` 的判定还在运行：state 3、距离 ≥100cm、信标方位角 (折叠到 ±90° 加 6° 偏置后) ≥15° 时置 1，而 `car_en` 在 `was_aligning==1` 期间恒为 0。
- Risk:
  - 由于无人机永远不会真的转过去把误差消掉，只要信标不在无人机机头/机尾 ±15° 扇区内且距离超过 1m，小车就会被持续急停；一旦几何关系变化使误差落回 15° 内，还会再触发 1 秒的 `car_en_disable_timer`。这解释了实车上"小车莫名停住不追"的现象。
- Suggested direction:
  - 既然旋转已停用，这整套 `was_aligning`/倒计时逻辑应一并禁用。

### 20. [FIXED] valid_track_cnt 不被 Mecanum_Stop/急停清理

- Evidence:
  - 旧实现使用 `valid_track_cnt` 作为视觉帧计数记忆，且不能被外部复位函数（如 `Visual_State_Reset`）可靠清理。
  - 当前实现已改为 `last_valid_track_time_ms` + `TRACK_MEMORY_MS = 1000U`，并在 `Visual_State_Reset()` 和过期路径中清理。
- Risk:
  - 修复前，看门狗急停又恢复通讯后，残留的"曾经有效跟踪"记忆允许小车在接收到一个孤立的 state 4（噪点或遮挡产生）数据包时，直接触发盲冲爆冲。
- Suggested direction:
  - 保持时间戳记忆路径，不再恢复帧计数锁定逻辑。

### 21. 盲冲早退屏蔽状态 3 恢复帧

- Evidence:
  - 小车端如果 `dash_end_time > 0`，`Visual_Control_Loop` 会直接 `return`。
- Risk:
  - 会屏蔽盲冲期间的状态 3 恢复帧，最长 600ms 不响应新视觉数据。
- Suggested direction:
  - 配合 -150ms 提前刹车可以接受，这是注释里写明的有意取舍。仅备案记录。

### 22. 解锁/锁定状态机存在多头控制（MCL_CAR）

- Evidence:
  - `Mecanum_Stop` 和 `Mecanum_Unlock` 被 4 个调用方交叉触发（ISR 内 IMU 校准检查、主循环看门狗、急停逻辑、无线调参通道8）。
  - 且存在 `uart_data[6]`、`has_unlocked`、`EN` 等状态的双重语义重叠。
- Risk:
  - 目前逻辑属于“碰巧自洽”，但状态迁移图非常脆弱。后续若调整架构，极易触发死锁或异常解锁。
- Suggested direction:
  - 作为技术债保留暂不修改。未来建议收敛为唯一的有限状态机 (FSM) 统一管理。

### 23. 强转指针遍历电机输出存在越界写风险（drone/fly_ctrl.c）

- Evidence:
  - 包含代码 `int16_t* motors = (int16_t*)&motor_out.rf;` 并在 `for (int i=0; i<4; i++)` 中进行限幅。
- Risk:
  - 高度耦合且依赖 `Motor_Output_t` 结构体中成员紧邻排布。若有任何人调整字段顺序或插入新成员，将导致严重的指针越界写（内存踩踏），大概率导致无人机空中 HardFault。
- Suggested direction:
  - 解除与结构体内存布局的隐性耦合，改为逐字段显式限幅。

### 24. 无人机紧急停机后，小车急停指令会被悬停任务覆盖（drone端）

- Evidence:
  - `cm7_0_isr.c` 中倾角 > 21° 时会触发 `car_en = 0; Flight_Lock();`。
  - 但在 `image_ctrl.c` 的 `Flight_Hover_Control_Task()` 中，每一帧都会重新执行 `if(was_aligning == 0) { car_en = 1; }`。
- Risk:
  - 紧急停机后视觉帧照常到来，`car_en` 在不到 20ms 后就会被重写回 `1` 并下发。导致无人机翻车锁桨后，小车却继续全速追踪，急停联动完全失效。
- Suggested direction:
  - 发生底层急停时设立一个不可恢复的全局标志位，在 `Flight_Hover_Control_Task` 或组装串口包处赋予最高优先级进行覆写。

### 25. 双写者缓存竞争 share_data_from_1 导致丢帧撕裂 (drone端)

- Evidence:
  - `data_complex.h` 注明该数组“Core 0 只读”，但 `main_cm7_0.c` 实际有写入（`S1_PROCESS_DONE=0`）并对整个 64 字节数组（横跨 2 条 32B cache line）执行 `SCB_CleanDCache_by_Addr`。
- Risk:
  - 存在严重的 Cache False Sharing 竞争窗口：CM7_0 `Invalidate` → 读取 → `Clean` 期间若 CM7_1 恰好写入新帧，CM7_0 的 `Clean` 会用自己 L1 Cache 中的旧帧数据暴力覆盖新帧，或把刚置 1 的 DONE 冲回 0。这会导致偶发丢帧或新旧坐标混拼的“撕裂帧”。撕裂发生在 UART 打包之前，小车端校验和无法拦截。
- Suggested direction:
  - 暂不修改。未来建议把握手标志移到独立的 cache line，或彻底改用 IPC 硬件信箱进行核间同步。

## Already Considered / Current Notes

- `duration_ms < 100` being clamped to 100ms is currently considered reasonable by the user.
- `Visual_Control_Loop()` cycle is about 50-60ms in the current setup, so any logic enforced only in that function should be interpreted with that timing, not the old 20ms comment.
- `origin/search_dev` contains a larger control refactor (`car_ctrl.c/h` and heavy `mecnum` changes). It was not merged into this checklist because the current task was to document known hazards on `codex_branch`, not to change control architecture.
