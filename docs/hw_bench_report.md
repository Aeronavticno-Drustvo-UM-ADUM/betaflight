# SATELLITEBOARD bench report: output check (2026-11-12)

Board: SATELLITEBOARD PCB, STM32F103, firmware `obj/main/betaflight_SATELLITEBOARD.elf` (flash verified equal to the ELF by OpenOCD `verify_image`; the ELF is newer than `servo_driver.c`). Setup: ST-Link V3 (SWD, OpenOCD) + UART1 over WCH adapter channel D (MAVLink, 115200). Nothing connected to the outputs, no USB to the board. Script: `test/tools/hw_output_check.py`.

## Method
- The script streams `ADUM_AUX_CHANNELS` at 50 Hz and reads the timer registers (PSC, CCR, TIM1 BDTR) over SWD. Pulse = CCR x (PSC+1) / 72 MHz. This proves what the firmware programs into the timers, not that the pin toggles (CCER, GPIO mode and the PCB are not checked). Stage 2: a logic analyzer on the pins.
- Motors are OneShot125: `pwmCompleteOneshotMotorUpdate` sets CCR to 0 after every update, so a plain read sees 0. The script halts the core on a hardware breakpoint at that function (CCRs valid there), reads, and resumes (about 1 ms).
- Heartbeat arm state: only a HEARTBEAT received after the command counts (HEARTBEAT is 1 Hz).

## Results
| Check | Result |
|---|---|
| Servos 1-5, two value sets, width vs command | PASS, error 0.0 us |
| Servo index -> pin | 1 PA15, 2 PB3, 3 PA8, 4 PA9, 5 PA10; same in both sets |
| TIM1 main output enable (MOE) | PASS |
| Motors disarmed, motor values in frames 1300-1500 | PASS, all 3 at 125.0 us (min_command 1000, OneShot125 / 8); PA6, PB0, PB1 |
| Arm edge (disarmed frames 1.5 s, then ARMED) accepted within 1.5 s | **FAIL**: see below |
| Armed motor widths, disarm, failsafe on link loss | not run (blocked by the arm failure) |

## Finding H1: arm edge not honoured on hardware, no STATUSTEXT
Disarmed frames for 1.5 s, then ARMED frames at 50 Hz: the board stays disarmed, 15 s in the last script run, 12 s and 14 s in earlier runs. Sometimes it arms late (2.7 s, 8.2 s). No `ARM REJECTED` STATUSTEXT in any case. Spec (`docs/arming_safety.md` sec 3): immediately on the arm edge, or a reject with reason.

Observed:
- Link clean: `drop_rate_comm` 0, `errors_comm` 0, `validFrames` 255, `haveCrcError` 0, `bootGraceDone` 1, `cliMode` 0, `saveBusy` 0, `sessionActive` 1.
- After the failure `prevFrameArmed` = 1 and `haveRejectText` = 0: the armed frames were processed, the edge check never produced a reject, and the state stayed DISARMED.
- First frame of a session saying ARMED (after a gap > 500 ms) arms at once, as specified. In one run the HEARTBEAT showed armed once, then disarmed within 1 s while armed frames kept coming.
- Arming also happened within 0.2-0.6 s in three runs where OpenOCD polled memory (`mdb`) every 0.1-0.7 s during the arm. An OpenOCD daemon attached without polling did not help. Cause unknown; the debugger access may change timing or stall the core briefly (which would cause a new session = unchecked arm).
- Not checked: whether SITL (`test/test_05_arming_safety.py`) shows the same delay; whether the first armed frame after the edge is dropped or processed with `newSession` wrong; the heartbeat path.

Reading of the code (`servo_driver.c` `servoDriverHandleAux`, around line 313-345): edge branch is `else if (!prevFrameArmed)` after `newSession`; a pass sets `SD_ARMED`, a reject sends STATUSTEXT. Neither is observed. A root cause needs on-chip tracing (breakpoint/watchpoint on `state` and `prevFrameArmed`, hardware stalls) or a UART-level capture.

## Bench facts learned
- `openocd -c init -c "mdb ..."` is non-halting; `halt` + breakpoint is needed for OneShot CCRs. Run only one OpenOCD at a time (kill stray daemons: `pkill openocd`).
- RCC_CSR reset flags stayed clear during the arm tests: no watchdog or other resets.
- Symbol addresses come from `obj/main/betaflight_SATELLITEBOARD.elf` with `tools/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-nm` (statics carry `.lto_priv.N` suffixes).
