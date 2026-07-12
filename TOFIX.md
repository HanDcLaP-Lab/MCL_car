# TOFIX

Verified unresolved defects in the current `MCL_car` and `../drone` working trees as of 2026-07-12.

Only reachable code defects and concrete control hazards are listed. Accepted behavior, parameter tuning, stale comments, disabled diagnostics, and speculative issues are omitted.

## P1 - Safety And Data Integrity

### DR-02. TOF failure leaves altitude control armed on indefinitely stale data

- Evidence:
  - VL53L8CX initialization/configuration/start return values are printed or ignored, but failure does not prevent later arming: `../drone/project/code/tof.c:136-160`, `../drone/project/code/fly_ctrl.c:116-121`.
  - Runtime ready-check and frame-read return statuses are ignored. Not-ready, failed, or zone-invalid frames return without invalidating height state or refreshing a health timestamp: `../drone/project/code/tof.c:180-192`.
  - `imu_data.z`, `imu_data.vz`, and `tof_base_throttle` retain their last values, and the 800Hz flight loop continues using the stale throttle: `../drone/project/code/tof.c:99-117`, `../drone/project/code/fly_ctrl.c:170-179`.
- Failure path: a sensor lockup, SPI fault, invalid-zone stream, or unsuccessful ranging start can leave the drone flying open-loop at the last collective output with no timeout, fallback, landing, or lock action.
- Impact: altitude can drift or accelerate until another independent protection triggers; stale height also corrupts visual ground projection and state4 distance decisions.
- Fix direction: track last valid TOF time and initialization health, gate arming on valid ranging, and enter an explicit bounded fallback/landing/lock state after a short freshness timeout.

### CR-05. Core0 writes back the Core1-owned vision cache lines

- Evidence:
  - `share_data_from_1` is a 64-byte Core1-to-Core0 buffer: `../drone/project/code/data_complex.c:14-24`, `../drone/project/code/data_complex.h:36-52`.
  - Core1 writes the frame and cleans the whole buffer: `../drone/project/user/main_cm7_1.c:90-93`.
  - Core0 invalidates it, writes `S1_PROCESS_DONE=0`, then cleans the same whole buffer: `../drone/project/user/main_cm7_0.c:127-142`.
  - Cortex-M7 cache lines are 32 bytes, so the acknowledgement shares a cache line with vision payload data.
- Failure path: Core1 can publish a newer frame after Core0 invalidates but before Core0 cleans. Core0 then writes its older payload cache lines back over that frame.
- Impact: a vision frame can be lost or internally mixed before it reaches hover control and the car; the later UART checksum cannot detect this corruption.
- Fix direction: keep each shared cache line single-writer. Put acknowledgement in a separate Core0-owned line or replace the flag with an IPC/mailbox ownership handoff.

## P2 - Runtime Control Defects

### CR-13. Synchronous VL53L8CX SPI work runs in a same-priority 1ms ISR

- Evidence:
  - PIT channel 0 calls `tof_update()` every 1ms: `../drone/project/user/cm7_0_isr.c:49-58`.
  - A ready frame is synchronously read and converted in that ISR: `../drone/project/code/tof.c:180-205`.
  - The platform performs blocking SPI array transfers, and the SPI driver busy-waits byte by byte: `../drone/project/code/vl53l8cx/platform.c:74-101`, `../drone/libraries/zf_driver/zf_driver_spi.c:572-594`.
  - PIT0 and the 800Hz flight PIT1 are both initialized at interrupt priority 3: `../drone/libraries/zf_driver/zf_driver_pit.c:163-204`.
- Impact: a TOF frame can delay a coincident attitude-control tick because the flight ISR cannot preempt it. The blocking path is certain; its worst-case duration still needs scope measurement.
- Fix direction: move frame transfer/processing out of the flight-critical ISR path, or use verified interrupt priorities/DMA so the 800Hz loop can preempt it.

### CR-14. TOF frame interval can truncate to zero after a long invalid gap

- Evidence:
  - The 32-bit tick difference is stored in `uint16_t`: `../drone/project/code/tof.c:22`, `../drone/project/code/tof.c:194-199`.
  - Invalid frames return before updating `tof_last_ready_tick`: `../drone/project/code/tof.c:189-192`.
  - The VL53L8CX path has no minimum `dt` clamp, and vertical speed divides by `dt`: `../drone/project/code/tof.c:101-103`, `../drone/project/code/tof.c:194-202`.
- Failure path: after at least 65.536s without a valid frame, elapsed time wraps modulo 65536ms; recovery can produce a very small or zero `dt`.
- Impact: zero `dt` can make vertical speed non-finite and poison the height PID state/output.
- Fix direction: keep elapsed time in `uint32_t`, apply minimum/maximum bounds before division, and reset derivative/filter state after a long invalid period.

### DR-03. Vision hold reprojects old pixels with the current attitude

- Evidence:
  - A missing beacon or car is marked valid for up to five frames while retaining its previous pixel center: `../drone/project/code/image.c:509-563`, `../drone/project/code/image.h:86-91`.
  - `calculate_ground_positions()` treats held `car_valid/target_valid` exactly like a current observation and projects those old pixels using the new frame's height, roll, pitch, and yaw, then updates the Kalman filters: `../drone/project/code/image_process.c:170-227`.
  - The held result drives hover control and is also downlinked to the car as a normal state1/3/4 coordinate: `../drone/project/code/image_ctrl.c:256-299`, `../drone/project/code/data_complex.c:136-152`.
- Impact: during drone rotation/tilt or height change, a stationary stale pixel becomes a fabricated ground displacement. This can command a false roll/pitch correction and can make the car chase a stale beacon instead of entering its own coast logic.
- Fix direction: keep raw-valid and held-valid semantics separate. Hold a ground-frame estimate without reprojecting it, and do not present held beacon data to downstream consumers as a fresh observation.

### DR-04. Motor mixer clips each motor independently and loses requested control ratios

- Evidence:
  - Collective, roll, pitch, yaw, and static offsets are summed independently for each motor: `../drone/project/code/fly_ctrl.c:218-236`.
  - Each result is then clamped separately to `[0, 8000]`: `../drone/project/code/fly_ctrl.c:238-243`, `../drone/project/code/fly_ctrl.h:10-14`.
  - Rate PID outputs can each reach 3500 while collective is around 5150 plus height/tilt compensation, so saturation is reachable during a large attitude/rate error: `../drone/project/code/fly_ctrl.c:55-69`, `../drone/project/code/fly_ctrl.c:170-179`.
- Impact: once one motor clips, differential torque and total thrust no longer match the controller request. Combined-axis corrections can lose roll/pitch authority or inject yaw/collective error exactly when recovery demand is highest.
- Fix direction: add mixer desaturation/collective shifting with explicit axis priority, and feed saturation information back to PID anti-windup.

### CR-07. A valid state4 can arm a timed dash with zero velocity

- Evidence:
  - `Visual_State_Reset()` clears `visual_last_vx/vy` and dash history but leaves `track_memory_ms` intact: `project/code/car_image.c:711-725`.
  - A coast expiry calls that reset without starting dash cooldown: `project/code/car_image.c:738-743`.
  - `State4_Handler()` checks only track memory and cooldown. If history estimation fails, it permits zero `dash_vx/vy`, changes only the divisor to `0.1f`, arms a 200-700ms timer, and publishes the unchanged zero vector: `project/code/car_image.c:655-700`.
  - While that timer is active, later nonzero visual commands are ignored: `project/code/car_image.c:790-795`.
- Impact: the car can stop near a valid beacon for the full dash interval, then continue after expiry, producing a repeatable stop-then-go discontinuity.
- Fix direction: require a recent nonzero command or a valid history estimate before arming a dash; otherwise avoid creating the dash timer.

### CR-15. The car main-loop watchdog automatically resumes an old command

- Evidence:
  - The 1ms ISR derives a local `armed` value from heartbeat age, but a timeout does not set a disarm reason or clear visual/velocity state: `project/code/mecnum.c:148-185`.
  - While the heartbeat is stale, PWM is forced to zero: `project/code/mecnum.c:279-299`.
  - On the next main-loop iteration, the heartbeat is refreshed before parsing/validation, so the ISR becomes armed again and ramps toward the still-stored `target_vel`: `project/user/main_cm4.c:88-98`, `project/code/mecnum.c:161-179`.
- Failure path: any operation lasting more than `MAINLOOP_STALL_MS=10ms` causes a temporary hard stop; completion of that operation automatically restarts the prior command even without a fresh visual frame.
- Impact: blocking print/parameter/parse work can create an unexplained periodic stop followed by continued motion, and a recovered software stall can resume stale movement.
- Fix direction: latch a stall disarm reason and clear motion state; require a newly validated control packet or explicit rearm before motion resumes.

## P3 - Narrow But Real Defects

### DR-05. Vision-loss timeout is based on main-loop iterations, not elapsed time

- Evidence: `vision_timeout_cnt` increments once per Core0 main-loop pass and trips at 400, while that loop also performs debug transport, parameter parsing, cache operations, UART transmission, and a 400us delay: `../drone/project/user/main_cm7_0.c:102-174`, `../drone/project/user/main_cm7_0.c:176-193`.
- Impact: after Core1 stops publishing, the duration for which the last roll/pitch target remains active varies with debug and communication load. It is neither 400ms nor otherwise bounded by a physical clock.
- Fix direction: store the last accepted vision timestamp from `dataC.pit0_cnt` and compare elapsed milliseconds.

### CR-16. Two-frame state0 debounce counts parser batches rather than received frames

- Evidence:
  - The UART parser drains all currently queued bytes and stores only the latest ordinary frame; only state4 and `car_en=0` receive dedicated event latches: `project/code/car_board_comm.c:83-100`, `project/code/car_board_comm.c:123-229`.
  - `zero_consecutive` increments only when `Visual_Control_Loop()` is called for the retained frame: `project/code/car_image.c:753-787`.
- Failure path: if two state0 packets arrive while the main loop is delayed, both can be collapsed into one call. A following nonzero packet resets the counter, so the documented two-frame state0 condition never fires.
- Impact: a short but valid two-frame all-lost event may fail to interrupt coast/dash after receive backlog.
- Fix direction: preserve a saturated consecutive-state0 count or stop event in the parser, then consume that event before normal visual processing.

### CR-17. Wireless yaw-rate tuning updates variables that the active PID never rereads

- Evidence:
  - Channels 1-3 update `YAW_RATE_KP/KI/KD`: `project/user/main_cm4.c:219-229`.
  - The main loop continuously copies wheel and yaw-hold gains into PID objects, but never copies the yaw-rate gains into `pid_yaw_rate`: `project/user/main_cm4.c:206-212`.
  - `pid_yaw_rate` receives those values only once during `Mecanum_Init()`: `project/code/mecnum.c:115-122`.
- Impact: the UI reports parameter updates, but yaw-rate tuning has no effect on the running controller, which can lead to false conclusions during stabilization work.
- Fix direction: apply the updated gains to `pid_yaw_rate` atomically at the same synchronization point as the other PID parameters.

### CR-18. Velocity commands are published to the 1ms ISR as a non-atomic tuple

- Evidence:
  - Main-loop visual code calls `Mecanum_Set_Velocity()`, which writes `target_vel.vx`, `vy`, and `wz` as three separate stores: `project/code/mecnum.c:96-100`.
  - The 1ms ISR reads those fields independently while updating the three smooth commands: `project/code/mecnum.c:161-179`.
  - The visual stop epoch prevents an ISR stop from being overwritten by an in-progress visual frame, but it does not prevent the ISR from sampling an ordinary direction update between component stores: `project/code/car_image.c:42-76`.
- Impact: one control tick can combine a new X component with an old Y/Z component. Acceleration limiting bounds the individual disturbance, but frequent direction changes can still inject avoidable wheel-target transients.
- Fix direction: publish a complete command through a sequence-checked double buffer or another short software snapshot protocol, then have the ISR consume only a coherent tuple.
