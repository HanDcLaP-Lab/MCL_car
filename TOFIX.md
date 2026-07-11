# TOFIX

Verified unresolved defects in the current `MCL_car` and `../drone` working trees as of 2026-07-10.

Only reachable code defects are listed. Resolved items, accepted behavior, parameter tuning, stale comments, and diagnostics-only limitations are omitted. Four issues remain: one P1 and three P2.

## P1 - Cross-Core Data Integrity

### CR-05. Core0 writes back the Core1-owned vision buffer cache lines

- Evidence:
  - `share_data_from_1` is documented as Core1-to-Core0 data: `../drone/project/code/data_complex.h:36-52`.
  - Core1 writes the frame, sets `S1_PROCESS_DONE`, and cleans the whole 64-byte array: `../drone/project/user/main_cm7_1.c:92-95`.
  - Core0 invalidates that array, writes `S1_PROCESS_DONE = 0`, and later cleans the whole array: `../drone/project/user/main_cm7_0.c:128-145`.
- Failure path: Core1 can publish a newer frame after Core0 invalidates its cache but before Core0 cleans it. Core0 then writes its older cache-line copy over the newer frame.
- Impact: a lost or internally torn vision frame can reach hover control and the car with a valid UART checksum.
- Fix direction: keep the data buffer single-writer. Put acknowledgement in a separate Core0-owned cache line or use an IPC/mailbox ownership handoff.

## P2 - Runtime Control Risks

### CR-13. Synchronous VL53L8CX SPI transfers run in a same-priority 1ms ISR

- Evidence:
  - PIT channel 0 calls `tof_update()` every 1ms: `../drone/project/user/cm7_0_isr.c:50-62`.
  - On every ready frame, it synchronously polls and reads the configured ranging payload, then converts it in the ISR: `../drone/project/code/tof.c:180-202`.
  - The SPI driver busy-waits per byte: `../drone/libraries/zf_driver/zf_driver_spi.c:572-594`.
  - PIT channel 0 and the 800Hz flight PIT channel 1 both use interrupt priority 3: `../drone/project/user/main_cm7_0.c:94-99`, `../drone/libraries/zf_driver/zf_driver_pit.c:163-193`.
- Impact: a ready TOF frame delays a coincident flight-control tick. The blocking and non-preemption are present; the exact delay still needs oscilloscope measurement.
- Fix direction: move the complete SPI read and processing outside the flight-critical interrupt path, or assign and verify a priority that lets flight control preempt it.

### CR-14. TOF frame interval can truncate to zero after a long invalid-data gap

- Evidence:
  - `dataC.pit0_cnt` and `tof_last_ready_tick` are 32-bit, but their difference is stored in `uint16_t dt_ticks`: `../drone/project/code/tof.c:22`, `../drone/project/code/tof.c:194-199`.
  - A frame with no valid trimmed distance returns before updating `tof_last_ready_tick`: `../drone/project/code/tof.c:189-199`.
  - The VL53L8CX path has no lower `dt` clamp, and vertical speed divides by `dt`: `../drone/project/code/tof.c:101-103`, `../drone/project/code/tof.c:194-202`.
- Failure path: after at least 65.536s without a valid trimmed distance, elapsed time truncates modulo 65536ms. Recovery can produce a very small or zero `dt`.
- Impact: zero `dt` can make `tof_vz_filt` non-finite and poison later height control.
- Fix direction: keep the subtraction in `uint32_t`, clamp minimum and maximum intervals before division, and reset derivative state after long invalid periods.

### CR-07. A valid `state4` can arm a timed dash with zero velocity

- Evidence:
  - Coast/dash expiry calls `Visual_State_Reset()`, which clears `visual_last_vx/vy` but not `track_memory_ms`: `project/code/car_image.c:716-726`.
  - After a long prior `state3`, a 600ms single-target coast can leave more than the 150ms lock threshold because memory decays instead of resetting: `project/code/car_image.c:177-215`, `project/code/car_image.c:547-555`.
  - `State4_Handler()` can then pass `Visual_Track_Is_Locked()` with zero residual velocity, clamp only its time-calculation divisor to `0.1f`, arm `dash_end_time`, and publish the unchanged zero vector: `project/code/car_image.c:660-701`.
  - While the timer is armed, later vision packets are ignored: `project/code/car_image.c:795-800`.
- Impact: after a coast expiry or similar reset, the car can remain stopped for 200-700ms near a valid beacon.
- Fix direction: require a recent nonzero command or a valid dash-history estimate before arming the dash; otherwise do not create a dash timer.
