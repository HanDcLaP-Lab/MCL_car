# TOFIX

Known hazards collected from Codex reviews through 2026-06-09.

Scope: `project/code`, `project/user`, and only the library behavior needed to explain project risks. This file is a working checklist, not a patch plan. Per project memory, stale comments after tuning and continuous parameter tuning should be mentioned but not treated as bugs by default.

## 2026-07-08 Current Cross-Repo Review

Current scope: the dirty working tree of both `MCL_car` and `../drone`. This section records current hazards only; no behavior code was changed for this review.

### P1 - Fix Or Verify Before More Field Runs

#### CR-01. `state0` two-frame debounce is currently ineffective on MCL_car

- Evidence:
  - `project/code/car_image.c:417-428` increments `zero_consecutive` and only returns when the count reaches 2.
  - On the first `locked_state == 0` frame, execution continues to `project/code/car_image.c:448-453`, where `case 0` still calls `State0_Handler()`.
  - `State0_Handler()` at `project/code/car_image.c:206-216` immediately stops, clears track memory, clears last velocity, and clears coast/merge state.
- Risk:
  - A single-frame all-lost packet still hard-stops the car despite the comment saying two consecutive frames are required. This can explain brief "slow down / stop / continue" symptoms when drone target validity flickers for one frame.
- Suggested direction:
  - Make the first zero frame explicitly skip `case 0` processing, or move the debounce result into the switch input so `State0_Handler()` is unreachable until the threshold is met.

#### CR-02. Drone `state4` semantics and car `state4` action are no longer aligned

- Evidence:
  - Drone now emits `locked_state=4` when raw car and raw target are both valid and raw relative distance is `<= 60cm`: `../drone/project/code/data_complex.c:136-139`, `../drone/project/code/data_complex.c:183-193`.
  - The car still treats `state4` as a near-fusion blind dash event: `project/code/car_image.c:313-345`.
  - `State4_Handler()` can start a hard-timed dash, block later vision packets while `dash_end_time > 0`, and then hard stop in the 1ms brake path: `project/code/car_image.c:320-337`, `project/code/car_image.c:430-435`, `project/code/car_image.c:371-377`.
- Risk:
  - `state4` has become a "near-distance event" while the receiver still handles it as "lamp fused / lamp-off event". In normal close approach with the beacon still visible, this can cause an unnecessary dash, a 1s cooldown, or a hard stop.
- Suggested direction:
  - Either restore `state4` to mean a real fusion/lamp-loss edge, or split the protocol so direct-near and fusion-dash are different states/actions.

#### CR-03. Short event states can be lost because the car stores only the latest parsed packet

- Evidence:
  - `Core_Parse_Board_Uart_Data()` drains all currently available FIFO bytes in one call and overwrites `uart_data` for every valid frame: `project/code/car_board_comm.c:80-87`, `project/code/car_board_comm.c:147-162`.
  - Completion is represented by one boolean `board_rx_complete_flag`, not a queue of accepted frames: `project/code/car_board_comm.c:9`, `project/user/main_cm4.c:106-137`.
  - Drone holds `state4` for only 3 image frames: `../drone/project/code/data_complex.c:138`, `../drone/project/code/data_complex.c:179-181`, `../drone/project/code/data_complex.c:188-190`.
- Risk:
  - If the car main loop is delayed by wireless printing, parameter parsing, FIFO bursts, or any stall, it can parse several drone packets and run `Visual_Control_Loop()` only once using the last one. A 2-3 frame event such as `state4` can therefore be missed even if the drone screen saw it.
- Suggested direction:
  - For short events, either latch them on the receiver until consumed, transmit them longer than the worst observed main-loop delay, or queue accepted UART packets instead of collapsing to the last packet.

#### CR-04. Drone emergency stop can be undone for `car_en` while the tilt stop remains active

- Evidence:
  - Tilt emergency in `../drone/project/user/cm7_0_isr.c:70-77` sets `car_en = 0` only inside `if (has_stopped == 0)`, then calls `Flight_Lock()` every time the angle remains excessive.
  - `Flight_Unlock()` unconditionally sets `car_en = 1`: `../drone/project/code/fly_ctrl.c:73-76`.
  - If an unlock command path runs after `has_stopped` is already 1, the next tilt ISR calls `Flight_Lock()` but does not force `car_en = 0` again.
- Risk:
  - The drone can remain locked due to tilt while the downlink says `car_en=1`, allowing the car to move after a drone-side emergency. This is a safety interlock issue.
- Suggested direction:
  - Make the emergency stop a latched high-priority state and derive `car_en` from it at packet assembly time, or always force `car_en=0` whenever `Flight_Lock()` is caused by an active emergency.

#### CR-05. `share_data_from_1` cache ownership race is still structurally present on drone

- Evidence:
  - `../drone/project/code/data_complex.h:36-52` documents `share_data_from_1` as Core 1 -> Core 0 data.
  - Core 0 nevertheless writes `S1_PROCESS_DONE = 0` and then cleans the entire 64-byte array: `../drone/project/user/main_cm7_0.c:128-143`.
  - Core 1 writes and cleans the same array after each image frame: `../drone/project/user/main_cm7_1.c:92-95`.
- Risk:
  - A Core 0 clean can write back stale cache-line data over a newer Core 1 frame. The car UART checksum cannot catch this because the torn data is already inside the packet payload.
- Suggested direction:
  - Put the handshake flag in its own cache line or replace this shared-memory protocol with a single-writer data buffer plus IPC/mailbox-style ownership transfer.

#### CR-06. UART1 TX and right-rear PWM still share P06_1 on MCL_car

- Evidence:
  - Board UART TX uses `UART1_TX_P06_1`: `project/code/car_board_comm.h:7-10`.
  - Right-rear motor PWM uses `TCPWM_CH00_P06_1`: `project/code/mecnum.h:27-32`.
  - Both `Board_Comm_Init()` and `Mecanum_Init()` initialize functions touching that pin: `project/code/car_board_comm.c:37-42`, `project/code/mecnum.c:102-113`.
- Risk:
  - Pin mux order can make UART TX or right-rear PWM unreliable. Even if current telemetry mostly uses RX, the duplicate init is fragile and misleading.
- Suggested direction:
  - Move one function to another pin and remove the temporary motor PWM init from board comm when hardware mapping is settled.

### P2 - Behavior And Robustness Risks

#### CR-07. `state4` can become a timed zero-velocity block if last visual speed is stale or zero

- Evidence:
  - `State4_Handler()` computes dash duration from `prev_car_dist / speed`, but clamps `speed` to `0.1f` if the last velocity is nearly zero: `project/code/car_image.c:322-328`.
  - It then starts `dash_end_time` and replays `visual_last_vx/vy`: `project/code/car_image.c:330-337`.
  - While `dash_end_time > 0`, `Visual_Control_Loop()` ignores all later visual states and returns early: `project/code/car_image.c:430-435`.
- Risk:
  - If `state4` arrives after a stop, a soft coast expiry, or before a good `state3` velocity, the car can enter a 100-700ms "dash" with zero command. This looks like a mysterious stop near the beacon.
- Suggested direction:
  - Require a minimum recent velocity for dash, or handle `state4` with no usable velocity as a normal tracking/stop state instead of arming `dash_end_time`.

#### CR-08. `state1/2` dash is closed, but persistent single-target state still hard-stops after 600ms

- Evidence:
  - `State12_Handler()` no longer starts dash, but always calls `Visual_Coast_With_Recovery()`: `project/code/car_image.c:220-227`.
  - `Visual_Coast_With_Recovery()` starts `visual_coast_end_time = now + COAST_HOLD_MS` and hard-stops at expiry: `project/code/car_image.c:154-199`.
  - `COAST_HOLD_MS` is 600ms: `project/code/mecnum.h:39-43`.
- Risk:
  - The fixed startup `state1/2` dash symptom is reduced, but a stable run with only one visible target can still coast for 600ms and then stop before resuming when `state3` returns.
- Suggested direction:
  - Decide whether persistent single-target should decay smoothly, hold, or stop. If the desired behavior is "do not hard stop while the only missing target is flickering", this branch needs a different policy.

#### CR-09. `visual_coast_expired` can inject a one-frame zero after recovery timing races

- Evidence:
  - The 1ms brake path clears state and then sets `visual_coast_expired = 1`: `project/code/car_image.c:378-384`.
  - The next `Visual_Coast_With_Recovery()` consumes the flag and immediately sends zero velocity: `project/code/car_image.c:154-160`.
- Risk:
  - If a target recovers around the coast expiry boundary, packet ordering can still force one zero-command frame. With the acceleration limiter and yaw loop, that can feel like a short hesitation.
- Suggested direction:
  - Clear or reinterpret `visual_coast_expired` when a stable `state3` frame has already resumed, or make the recovery path consume the flag without forcing zero when current data is valid.

#### CR-10. Jump detection still compares absolute target coordinates, not target-relative-to-car coordinates

- Evidence:
  - `State3_Handler()` detects a beacon jump with `uart_data[2/3] - prev_target_x/y`: `project/code/car_image.c:268-280`.
  - It does not compare `(target - car)` vectors, even though both car and target are downlinked in the drone-relative coordinate frame.
- Risk:
  - Drone motion, yaw projection changes, or car-following drift can move the absolute target coordinate even when the beacon identity did not change. Conversely, a target switch that is partly masked by drone motion can be less obvious than it is relative to the car.
- Suggested direction:
  - If this branch remains important, compare the raw relative vector `(target_x - car_x, target_y - car_y)` frame to frame, ideally using the same unfiltered/raw semantics chosen for drone fusion detection.

#### CR-11. Direct-distance `state4` uses raw projection without debounce or hysteresis

- Evidence:
  - Drone checks `pos.raw_target - pos.raw_car` directly against `60cm`: `../drone/project/code/data_complex.c:183-189`.
  - Raw projection depends on image classification plus height/pitch/roll snapshot from the same frame: `../drone/project/code/image_process.c:151-164`, `../drone/project/user/main_cm7_1.c:80-89`.
- Risk:
  - Near the threshold, one noisy height/attitude/image frame can produce a `state4` rising edge. Because the car treats `state4` as dash/coast, a single noisy edge has a large behavioral effect.
- Suggested direction:
  - Add a small debounce/hysteresis or require a stronger fusion/loss signal if `state4` continues to drive blind motion.

#### CR-12. Drone low-height protection resets `state4` cooldown and can emit hard state0

- Evidence:
  - If `img_imu_snap.height < 90cm` for 5 image frames, `M7_1_data_send()` clears `fusion_state4_hold_frames`, clears the last trigger timestamp, emits `S1_LOCKED_COUNT = 0`, and returns: `../drone/project/code/data_complex.c:140-173`.
  - The 5-frame comment assumes about 100ms: `../drone/project/code/data_complex.c:141`.
- Risk:
  - A TOF dip or transient low-height estimate can force the car into all-lost handling and also reset the 1s `state4` cooldown, allowing an immediate retrigger after height recovers.
- Suggested direction:
  - Treat low-height protection as a separate downlink state or preserve the cooldown timestamp across low-height invalidation unless immediate retrigger is intentional.

#### CR-13. TOF SPI polling and data read run inside the 1ms drone ISR

- Evidence:
  - CM7_0 PIT0 1ms ISR calls `tof_update()`: `../drone/project/user/cm7_0_isr.c:50-59`.
  - In VL53L8CX mode, `tof_update()` polls readiness and reads the ranging data over SPI when ready: `../drone/project/code/tof.c:180-202`.
- Risk:
  - A blocking SPI read in an ISR can add jitter to other time-critical control work. Around TOF frame boundaries this can couple height updates, control timing, and the observed small altitude drops.
- Suggested direction:
  - Measure ISR execution time around ready frames; if it is nontrivial, move heavy SPI reads to a lower-priority task and let the ISR only timestamp/flag readiness.

#### CR-14. VL53L8CX TOF path removed the lower `dt` clamp

- Evidence:
  - VL53L8CX branch computes `dt` from `dataC.pit0_cnt - tof_last_ready_tick`; the old lower fallback is commented out: `../drone/project/code/tof.c:194-202`.
  - DL1B branch still clamps `dt < 0.005f` to 20ms: `../drone/project/code/tof.c:212-220`.
- Risk:
  - A first-frame or very short interval can feed an extremely small `dt` into height PID derivative/integral logic. Output is limited, but a one-frame throttle step is still possible.
- Suggested direction:
  - Keep a consistent lower-bound policy for both TOF backends and explicitly initialize `tof_last_ready_tick` on sensor start.

#### CR-15. MCL_car main-loop watchdog threshold may be too tight for diagnostic load

- Evidence:
  - `MAINLOOP_STALL_MS` is 10ms: `project/code/mecnum.h:61-65`.
  - The 1ms ISR treats a stale heartbeat as unarmed and zeros smoothed velocity: `project/code/mecnum.c:148-185`.
  - Main loop can parse UART bursts, run SeekFree assistant parsing, and send wireless confirmation strings: `project/user/main_cm4.c:98-162`.
- Risk:
  - A short debug print or parsing burst over 10ms can look like a safety stop even though communication and vision are healthy. This can create intermittent stops that are hard to distinguish from visual state changes.
- Suggested direction:
  - Use `visual_loop_max_dt_debug`/comm debug to measure real worst-case loop time with the exact wireless diagnostics enabled, then tune the watchdog threshold or split slow diagnostics.

#### CR-16. Lower PWM cap changed saturation behavior without retuning the wheel PID envelope

- Evidence:
  - PWM max is now 5000: `project/code/mecnum.h:12`, `project/code/mecnum.c:58-61`.
  - `OUT_MAX` follows the cap, but `KP=3500`, `KI=20000`, `MAX_I=4500` remain aggressive: `project/code/mecnum.c:6-12`, `project/code/mecnum.h:25`.
  - Equal-proportion scaling and anti-windup write back capped motor outputs: `project/code/mecnum.c:268-277`.
- Risk:
  - More commands will sit on the output cap. This can reduce acceleration/turn authority, change braking feel, and make heading correction fight translational commands differently from the old 7000 cap.
- Suggested direction:
  - Retune or at least re-log wheel output saturation rate after the 5000 cap; update docs that still mention 7000.

#### CR-17. Acceleration limiting is component-wise, not vector-norm based

- Evidence:
  - The 1ms loop limits `vx`, `vy`, and `wz` independently: `project/code/mecnum.c:161-179`.
  - Visual commands keep constant speed magnitude but can rotate direction abruptly: `project/code/car_image.c:291-300`.
- Risk:
  - Direction changes do get limited because both components ramp, but diagonal vector acceleration can be up to about `sqrt(2)` times the per-axis limit. The yaw PID correction also bypasses `smooth_wz` when holding heading.
- Suggested direction:
  - If side-slip or flip risk remains during rapid direction changes, consider vector-norm acceleration limiting for `(vx, vy)` and log the commanded-vs-smoothed vector angle.

#### CR-18. `Visual_Track_Is_Locked()` only accumulates during `state3`

- Evidence:
  - `Visual_Track_Refresh()` is called in stable `State3_Handler()` and in active merge coast: `project/code/car_image.c:253-255`, `project/code/car_image.c:308-309`.
  - `State12_Handler()` decays memory: `project/code/car_image.c:220-227`.
  - `State4_Handler()` requires `track_memory_ms >= 150ms`: `project/code/car_image.c:143-145`, `project/code/car_image.c:320`.
- Risk:
  - A direct-distance `state4` that arrives early in a run, or after prolonged single-target tracking, will not dash and will fall into soft coast. That can look like a near-beacon hesitation even though the drone is intentionally emitting `state4`.
- Suggested direction:
  - Decide whether direct-near `state4` should require 150ms of prior `state3`; if not, direct-near needs a separate confidence rule.

#### CR-19. Hard-coded dash clamp does not use the configured dash macros

- Evidence:
  - `DASH_MS_MIN` and `DASH_MS_MAX` are defined as 200 and 650: `project/code/mecnum.h:51-55`.
  - `State4_Handler()` hard-clamps to 100 and 600, then adds `STATE4_DASH_EXTRA_MS`: `project/code/car_image.c:328-333`.
- Risk:
  - The actual state4 dash range is 200ms to 700ms with the current `+100ms`, while the macro names suggest a different range. Tuning via macros will not do what the reader expects.
- Suggested direction:
  - Use the macros or rename/comment them as stale until the state4 experiment settles.

#### CR-20. Drone target hold can mask real signal loss for about 100ms

- Evidence:
  - `apply_target_hold_logic()` holds target validity for `TARGET_HOLD_FRAMES` after at least 3 consecutive target frames: `../drone/project/code/image.c:510-536`.
  - `TARGET_HOLD_FRAMES` is 5 and image processing is about 50Hz: `../drone/project/code/image.h:86-91`, `../drone/project/user/main_cm7_1.c:72-75`.
  - `locked_count` uses held `cam_down.target_valid`, while `raw_locked_count` separately tracks true raw visibility: `../drone/project/code/data_complex.c:143-149`.
- Risk:
  - After the beacon really disappears, the downlink can continue reporting state3 briefly. This is useful against flicker, but it delays the car's single-target/coast behavior and can confuse field diagnosis if the screen cross is based on held validity.
- Suggested direction:
  - Keep this if flicker tolerance is desired, but display/log raw validity separately when diagnosing lamp-off and state4 behavior.

#### CR-21. Multi-beacon selection can jump instantly now that switch-confirm logic is gone and distance bonus is zero

- Evidence:
  - Drone target selection chooses the candidate with minimum distance to the car, or to the image center if no car is found: `../drone/project/code/image.c:733-778`.
  - The hysteresis match radius remains 80cm, but the linear bonus is `0.0f`: `../drone/project/code/image.h:93-99`.
  - `TARGET_SWITCH_CONFIRM_FRAMES` remains defined but is not used in `sort_lights()`: `../drone/project/code/image.h:99`, `../drone/project/code/image.c:563-805`.
- Risk:
  - In multi-beacon scenes, small classification/projection changes can switch the selected beacon immediately. The car then sees a target jump and may enter merge coast or stop, especially if residual velocity is zero.
- Suggested direction:
  - If instant switching is desired, keep it and rely on car-side merge handling. Otherwise reintroduce a simpler same-target confidence rule or a nonzero hysteresis bonus.

#### CR-22. Drone `dataC.pit0_cnt` has different meanings on the two CM7 cores

- Evidence:
  - CM7_0 increments `dataC.pit0_cnt` every 1ms: `../drone/project/user/cm7_0_isr.c:50-59`.
  - CM7_1 increments its own compiled instance by 10 every 10ms: `../drone/project/user/cm7_1_isr.c:64-69`.
  - Both use the same struct field name for timing-sensitive logic and display markers: `../drone/project/code/data_complex.h:54-59`, `../drone/project/code/display.c:123-136`.
- Risk:
  - This is probably separate per-core storage, but the shared name invites future mistakes. A developer may assume cooldowns, display markers, and flight timers share one global clock when they do not.
- Suggested direction:
  - Rename or comment the field as per-core local time, or split into explicit CM7_0/CM7_1 timer fields.

### P3 - Diagnostics, Documentation, And Lower-Priority Issues

#### CR-23. `merge` wireless print is currently disabled, so screen marker is the only active state4 indicator

- Evidence:
  - CM7_0 sets `merge_print_pending` on a `state4` rising edge: `../drone/project/user/main_cm7_0.c:135-139`.
  - Actual wireless transmission is commented out: `../drone/project/user/main_cm7_0.c:177-180`.
  - Screen marker reads the shared `S1_LOCKED_COUNT` on CM7_1 display side: `../drone/project/code/display.c:123-136`, `../drone/project/code/display.c:201-205`.
- Risk:
  - "Drone displayed merge" and "car received state4" are not equivalent observations because CM7_0 cache handoff, UART send, car FIFO collapse, and receiver timing are in between.
- Suggested direction:
  - For state4 debugging, log the car-side received `uart_data[5]` or latch a receiver-side event counter. Keep wireless printing disabled during timing-sensitive runs unless measuring its overhead.

#### CR-24. Several protocol and tuning comments are stale

- Evidence:
  - `project/code/car_image.h:10-16` lists `locked_state` only through 3.
  - `../drone/README.md:189-194` also lists locked states only through 3.
  - `project/code/AGENTS.md:19-38` says `PWM_MAX_M = 7000.0f`, while code is 5000.
  - `project/user/AGENTS.md:24-35` says PIT_CH1 is 25ms, while `main_cm4.c` initializes it to 200ms: `project/user/main_cm4.c:74`.
- Risk:
  - Future tuning and diagnosis can be based on old semantics, especially around `state4`, PWM saturation, and debug timing.
- Suggested direction:
  - Update docs/comments after state4 semantics are finalized. Do not treat pure stale comments as behavior bugs unless they affect a field run.

#### CR-25. Old TOFIX entries 24 and 17 are stale and should not be applied blindly

- Evidence:
  - Current `Flight_Hover_Control_Task()` no longer writes `car_en = 1`; it only comments that visual no longer controls car enable: `../drone/project/code/image_ctrl.c:230-245`.
  - Current `wireless_uart_get_()` no longer calls `Mecanum_Stop()`; it only echoes received wireless bytes: `project/code/wireless_uart.c:20-31`.
- Risk:
  - Following old checklist text literally would send debugging in the wrong direction.
- Suggested direction:
  - Keep old entries as history, but use this 2026-07-08 section as the current truth unless later code changes invalidate it.

#### CR-26. MCL_car yaw-rate wireless tuning still does not update the active `pid_yaw_rate`

- Evidence:
  - `Wireless_Update()` changes `YAW_RATE_KP/KI/KD`: `project/user/main_cm4.c:206-216`.
  - The main loop syncs motor PID and `pid_yaw_hold`, but not `pid_yaw_rate`: `project/user/main_cm4.c:195-199`.
  - `Mecanum_Control_Loop()` uses `pid_yaw_rate` for the inner yaw loop: `project/code/mecnum.c:207-218`.
- Risk:
  - Wireless channels 1-3 appear to tune yaw rate but may not affect the running PID object until restart or manual code change.
- Suggested direction:
  - Sync `pid_yaw_rate` parameters in the same place as `pid_yaw_hold`, or tune the PID object directly.

#### CR-27. MCL_car yaw hold still targets global zero, not a captured heading

- Evidence:
  - First IMU update initializes `yaw_total` to the current yaw: `project/code/imu_car_rc.c:40-60`.
  - Yaw hold computes `0.0f - imu_car_rc_data.yaw_total`: `project/code/mecnum.c:196-208`.
- Risk:
  - If the car starts at a nonzero yaw and the intended behavior is "hold current heading", the control loop will rotate toward global zero while chasing the beacon.
- Suggested direction:
  - Confirm the intended yaw convention. If hold-current-heading is desired, store a yaw reference on unlock or before visual mode starts.

#### CR-28. IMU car pitch assignment can leave stale values

- Evidence:
  - `imu_car_rc_data.pitch` is assigned only when `imu660rc_roll > 90` or `< -90`: `project/code/imu_car_rc.c:22-25`.
  - For the common `[-90, 90]` range there is no `else`, so the previous pitch value remains.
- Risk:
  - Pitch debug and any future pitch-based safety logic can read stale data.
- Suggested direction:
  - Add an explicit normal-range assignment or document that pitch is intentionally unused.

#### CR-29. `OUT_MAX` macro should be parenthesized

- Evidence:
  - `project/code/mecnum.h:25` defines `#define OUT_MAX PWM_MAX_M`.
- Risk:
  - Low current risk because it is used as a simple argument, but it is fragile if later used inside an expression.
- Suggested direction:
  - Prefer `#define OUT_MAX (PWM_MAX_M)` during the next nearby cleanup.

#### CR-30. Drone motor output limiting relies on struct field layout

- Evidence:
  - `Flight_Motor_Mix()` casts `&motor_out.rf` to `int16_t*` and loops 4 values: `../drone/project/code/fly_ctrl.c:238-243`.
  - `Motor_Output_t` currently places `rf/rb/lb/lf` contiguously after three floats: `../drone/project/code/fly_ctrl.h:61-71`.
- Risk:
  - This works only while the struct layout stays exactly as expected. A future field insertion or reorder can silently limit the wrong memory.
- Suggested direction:
  - Replace with explicit per-field clamps.

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
