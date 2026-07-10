# TOFIX

Known hazards collected from Codex reviews through 2026-07-10.

Scope: `project/code`, `project/user`, and only the library behavior needed to explain project risks. This file is a working checklist, not a patch plan. Per project memory, stale comments after tuning and continuous parameter tuning should be mentioned but not treated as bugs by default.

## 2026-07-08 Current Cross-Repo Review

Current scope: the dirty working tree of both `MCL_car` and `../drone`. CR-01 was fixed on 2026-07-10; other entries remain review findings unless explicitly marked resolved.

### P1 - Fix Or Verify Before More Field Runs

#### CR-01. Resolved: MCL_car `state0` now requires two consecutive packets

- Resolution (2026-07-10):
  - The first `locked_state == 0` packet now returns before the state switch, preserving the previous command.
  - The second consecutive packet executes `State0_Handler()` and resets all visual motion state.
  - The saturating debounce counter is also reset by `Visual_State_Reset()`, so packets separated by a disarm/reconnect cannot be combined.

#### CR-02. Drone `state4` semantics and car `state4` action are no longer aligned

- Evidence:
  - Drone now emits `locked_state=4` when raw car and raw target are both valid and raw relative distance is `<= 60cm`: `../drone/project/code/data_complex.c:136-139`, `../drone/project/code/data_complex.c:183-193`.
  - The car still treats `state4` as a near-fusion blind dash event: `project/code/car_image.c:313-345`.
  - `State4_Handler()` can start a hard-timed dash, block later vision packets while `dash_end_time > 0`, and then hard stop in the 1ms brake path: `project/code/car_image.c:320-337`, `project/code/car_image.c:430-435`, `project/code/car_image.c:371-377`.
- Risk:
  - `state4` has become a "near-distance event" while the receiver still handles it as "lamp fused / lamp-off event". In normal close approach with the beacon still visible, this can cause an unnecessary dash, a 1s cooldown, or a hard stop.
- Suggested direction:
  - Either restore `state4` to mean a real fusion/lamp-loss edge, or split the protocol so direct-near and fusion-dash are different states/actions.

#### CR-03. Resolved: short state4 and stop events are latched while draining the FIFO

- Resolution (2026-07-10):
  - The parser still drains to the latest continuous frame, but preserves the most recent complete state4 frame from the batch.
  - Any `car_en=0` in the batch is stop-dominant and cannot be overwritten by a later `car_en=1`; re-enable requires a subsequent fresh batch.
  - A following state0 cancels state4, and state4 followed by more than five ordinary frames is discarded as stale.
  - UART FIFO write failures are counted separately so byte loss can be distinguished from event collapsing.

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

### P2 - Behavior And Robustness Risks

#### CR-31. Main-loop velocity writes can race the 1ms timer stop

- Evidence:
  - Main-loop visual control publishes a command through `Mecanum_Set_Velocity()`, which writes `target_vel.vx/vy/wz` as three independent stores: `project/code/mecnum.c:96-100`.
  - Active dash/coast/merge paths can replay a nonzero command from the main loop while their absolute timer is still armed: `project/code/car_image.c:433-460`, `project/code/car_image.c:519-567`, `project/code/car_image.c:746-751`.
  - The 1ms ISR calls `Visual_Brake_Check()`, whose dash/coast/merge expiry paths write the same three fields to zero: `project/code/mecnum.c:155-156`, `project/code/car_image.c:681-707`.
  - Individual aligned 32-bit stores are atomic on this MCU, but the three-field command has no critical section, sequence counter, or single-writer handoff.
- Trigger window:
  - The main loop enters `Mecanum_Set_Velocity()`, the timer ISR interrupts around its three stores, zeros the command and clears the timer, then the main loop resumes and writes some or all command fields after the stop.
  - The same stale-write window also exists if the main loop decides a timer is still active, the ISR expires it before the setter is called, and the interrupted main loop then publishes the already-decided command.
  - The window is only a few instructions around one timer-expiry edge. It is a genuine race, but not a continuously active fault.
- Risk:
  - With healthy vision traffic, the residual command normally lasts no longer than the next packet interval. At `0.75m/s`, a 20-60ms interval corresponds to roughly 1.5-4.5cm before accounting for wheel dynamics.
  - If packets also stop at that moment, the partial or full command can remain until the roughly 1000ms communication watchdog. The race alone does not explain deterministic multi-second sliding; that would require another state-machine or communication failure as well.
- Why one dirty bit is insufficient:
  - If the main loop marks the command dirty and the ISR simply skips its write, the lower-priority motion write wins and the safety stop is lost.
  - `if (!dirty) dirty = 1` is also a check-then-set sequence. The ISR can run between the check and assignment, finish the stop, and then return to a main loop that still writes the stale command.
- Suggested direction:
  - A related nonblocking design can work with two separate flags: `velocity_write_busy` plus a stop-dominant `velocity_stop_pending`. The ISR always latches `stop_pending`; if a write is busy it leaves the timer/event pending and retries on the next 1ms tick. The main writer checks the stop latch before publishing, never clears it, and a fresh visual-frame boundary explicitly acknowledges it.
  - The simpler recommended design is a stop latch plus a very short, nesting-safe critical section around the latch check and all three stores. Set the latch before the ISR zeros velocity; clear it only when a genuinely new visual frame is allowed to command motion. Use `Cy_SysLib_EnterCriticalSection()`/`Cy_SysLib_ExitCriticalSection()`, not the project's counter-based `interrupt_global_disable()` wrapper, which can re-enable interrupts from a nested critical section.

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

#### CR-11. Resolved: direct-distance state4 uses a 3-frame raw-distance median

- Resolution (2026-07-10):
  - Only consecutive raw dual-target frames at normal height enter the window; any invalid or low-height frame resets it.
  - The median of the latest three raw Euclidean distances is compared with 60cm, rejecting one-frame projection spikes while adding about one frame of delay during steady approach.
  - Low-height protection clears this distance window so recovery requires three fresh frames.
  - Existing one-second cooldown and sustained-near retrigger semantics were intentionally left unchanged.

#### CR-12. Drone low-height protection resets `state4` cooldown and can emit hard state0

- Evidence:
  - If `img_imu_snap.height < 90cm` for 5 image frames, `M7_1_data_send()` clears `fusion_state4_hold_frames`, clears the last trigger timestamp, emits `S1_LOCKED_COUNT = 0`, and returns: `../drone/project/code/data_complex.c:140-173`.
  - The 5-frame comment assumes about 100ms: `../drone/project/code/data_complex.c:141`.
- Risk:
  - A TOF dip or transient low-height estimate can force the car into all-lost handling and also reset the 1s `state4` cooldown, allowing an immediate retrigger after height recovers.
- Suggested direction:
  - Cancel the active state4 hold at low height, but preserve the trigger timestamp and near-zone latch.
  - Add height hysteresis (`<90cm` enter, `>100cm` exit, both confirmed for 5 frames) plus a separate 1000ms recovery cooldown.
  - Recovery must not automatically re-arm state4 while the target remains inside the near zone.

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

#### CR-19. Resolved: dash clamp now uses the configured final-time macros

- Resolution (2026-07-10):
  - `DASH_MS_MIN/MAX` now explicitly describe the final range after fixed compensation: 200ms to 700ms.
  - `State4_Handler()` adds `STATE4_DASH_EXTRA_MS` before clamping and uses the macros directly.

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

## Already Considered / Current Notes

- `duration_ms < 100` being clamped to 100ms is currently considered reasonable by the user.
- `Visual_Control_Loop()` cycle is about 50-60ms in the current setup, so any logic enforced only in that function should be interpreted with that timing, not the old 20ms comment.
- `origin/search_dev` contains a larger control refactor (`car_ctrl.c/h` and heavy `mecnum` changes). It was not merged into this checklist because the current task was to document known hazards on `codex_branch`, not to change control architecture.
