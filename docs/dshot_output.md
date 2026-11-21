# DShot output on the satellite (plan)

Design note and agent plan, 2026-11-21. Status: **planned, nothing implemented**. Readers: the owner and the agents that
implement it (see section 8). Line numbers refer to `c934d2163` and may drift; search for the quoted code if they do.

## 1. Owner decisions (2026-11-21)

| # | Decision |
|---|---|
| DS-D1 | Motors M1-M3 move from standard PWM 400 Hz to **DShot, output only**: no bidirectional DShot, no ESC telemetry, no DShot commands (beacon, direction). Reason: no ESC calibration, identical throttle on all 3 ESCs, explicit motor stop, checksum, at low risk with mostly proven Betaflight code. |
| DS-D2 | Protocol **DShot300**. DShot150 is the fallback if the long wires to the wing ESCs cause problems. |
| DS-D3 | Standard PWM stays available as a fallback: one line in `target/SATELLITEBOARD/config.c`. |
| DS-D4 | Reuse Betaflight's own code: the STM32F3 timer-DMA DShot driver, built for F1. This **approves changes to upstream driver files** (`CLAUDE.md`: "do not modify upstream Betaflight code beyond the `USE_SERVO_DRIVER` paths without asking"), limited to adding F1 next to the existing F3 branches. No new driver code. |
| DS-D5 | INAV does not change: it keeps `motor_pwm_protocol = STANDARD` and sends 1000-2000 us. The satellite converts. |

Not chosen (for the record): bidirectional DShot (shares the throttle output path with new F1-only GPIO direction code;
the 22-33 ohm series resistors on the outputs do not protect against a drive conflict, ~100 mA vs 25 mA pin limit);
serial ESC telemetry (no telemetry pad on the Raptor5 ESC, no free UART); OneShot125 (no benefit over 400 Hz PWM here).

## 2. Hardware facts

| Item | Value | Source |
|---|---|---|
| ESC | Flycolor Raptor5 G071-35A, BLHeli_32 ("Flycolor_Raptor_5"), STM32G071. "All Dshot and Proshot signals are supported", protocol auto-detected at power-up, throttle calibration ignored with DShot | Flycolor manual |
| Motor pins | M1 PA8 TIM1_CH1 (O4), M2 PA9 TIM1_CH2 (O6), M3 PA10 TIM1_CH3 (O5) | `target/SATELLITEBOARD/target.c:40-42` |
| DMA for those channels | DMA1 Ch2, Ch3, Ch6 | `drivers/timer_def.h:169-171` |
| Other DMA users | UART1 TX/RX DMA on DMA1 Ch4/Ch5 (`common_pre.h:33-35`). UART2 RX DMA would be Ch6 and UART3 RX Ch3: both must stay interrupt-driven (they are) | |
| Burst DShot (DMAR) | **Not usable**: TIM1_UP = DMA1 Ch5 = UART1 RX | |
| Timer clock | TIM1 72 MHz. DShot300 = 6 MHz tick (prescaler 12), 20 ticks per bit, ARR 19; DShot150 prescaler 24 | `drivers/dshot_dpwm.h:28-34`, `pwm_output_dshot.c:325-326` |
| Series resistors | 22-33 ohm on the outputs (owner, not measured). RC time constant a few ns vs 1.25 us shortest high time: no effect | |
| Budget | Flash 68.6 % of 128 KB (2026-11-21), RAM 20 KB | ESC-RATE build |

## 3. What changes in the firmware

### 3.1 Build the F3 DShot driver for F1 (upstream files, DS-D4)

| File | Change | Why |
|---|---|---|
| `target/SATELLITEBOARD/target.h` | `#define USE_DSHOT`; explicitly `#undef USE_DSHOT_DMAR`, `USE_DSHOT_TELEMETRY`, `USE_DSHOT_BITBANG`, `USE_ESC_SENSOR` | DShot on this target only, nothing else |
| `make/mcu/STM32F1.mk` (`MCU_COMMON_SRC`, line 63) | add `drivers/pwm_output_dshot.c`, `drivers/pwm_output_dshot_shared.c` | F3 lists them (`STM32F3.mk:88-89`); `dshot.c`, `dshot_dpwm.c`, `dshot_command.c` are already common (`make/source.mk:15-17`) |
| `drivers/pwm_output_dshot.c`, `pwm_output_dshot_shared.c` | header `stm32f10x.h` next to `stm32f30x.h` (lines 39-43 / 40-44); every `#if defined(STM32F3)` DMA branch (`DMA_DIR_PeripheralDST`, `DMA_M2M_Disable`, lines 101, 109) also for F1; `CCM_CODE` (lines 123, 191) only on F3, F1 gets `FAST_CODE` | Same DMA controller design and SPL calls on F1 and F3 |
| `drivers/pwm_output_dshot.c:315-316` | F1 branch: `IOConfigGPIO(motorIO, IOCFG_AF_PP)` instead of `IO_CONFIG(GPIO_Mode_AF, ...)` + `IOConfigGPIOAF(..., alternateFunction)` | F1 has no AF number (`timer.h:94`); same pattern as `pwm_output.c:237` |
| `drivers/dshot_dpwm.h:74` | add `STM32F1` to the `uint32_t` buffer list | F1 falls to `uint8_t`, but the DMA is set to word size (`pwm_output_dshot.c:394-395`, `419-420`): mismatch. F1 timer registers accept word access |
| Anything else that fails to compile | Same rule: add F1 where F3 is, never new logic. List every change in the report | |

Checks the agent must confirm in code (not assume):
- `pwm_output_dshot.c:457` `TIM_CtrlPWMOutputs(timer, ENABLE)` runs for TIM1 (MOE; the bench pin test checks it too).
- `timerHardware->dmaRef` is filled for F1 by `DEF_TIM` (`timer_def.h:146`ff, no `USE_DMA_SPEC` on F1).
- The DMA1 Ch2/Ch3/Ch6 IRQ handlers exist for F1 (`dma.c:36-42`) and nothing else claims them.
- Nothing in servo-driver mode sends DShot commands (`dshot_command.c`); the `pwmCompleteDshotMotorUpdate` command queue stays empty.

### 3.2 Protocol-neutral motor values (`io/servo_driver.c`, our code)

Today `servoDriverWriteMotors()` (`servo_driver.c:326-337`) passes microseconds straight into `motorWriteAll()`. That is
correct only for PWM. Change: convert each value with Betaflight's `motorConvertFromExternal()` (`motor.c:161`) before
`motorWriteAll()`.

| Protocol | `motorConvertFromExternal(us)` | Effect |
|---|---|---|
| PWM (`pwm_output.c:164`) and SITL (`target/SITL/target.c:453`) | identity | **No behaviour change** for PWM and SITL: the SITL suite must pass unchanged |
| DShot (`dshot.c:81`) | 1000 -> `DSHOT_CMD_MOTOR_STOP` (0); 1001-2000 -> 48-2047 linear | Disarmed, failsafe, boot grace (`mincommand` 1000) send stop frames. Armed 1000 (rear motor off in FW) = stop. INAV owns idle (`throttle_idle`); `digitalIdleOffsetValue` is not used on this path |

- `motor[i]` (MSP monitoring, `servo_driver.c:335`) keeps the **microsecond** value, so `sat_usb_monitor.py` stays valid.
- Frames go out on every 200 Hz cycle in every state (TODO A). The ESC needs this continuous stream.

### 3.3 Switch the protocol (`target/SATELLITEBOARD/config.c:59-63`)

`motorPwmProtocol = PWM_TYPE_DSHOT300`. `motorPwmRate` no longer applies (DShot is written per update). Keep a comment
with the one-line fallback to `PWM_TYPE_STANDARD` / 400 Hz. `mincommand` 1000 and `maxthrottle` 2000 stay (they are the
external range).

## 4. Spec changes (`docs/arming_safety.md`, new section 15 "Motor protocol")

| # | Rule |
|---|---|
| DS1 | DISARMED, FAILSAFE and boot grace output DShot `MOTOR_STOP` frames continuously (every cycle), never silence. |
| DS2 | ARMED and MOTOR_TEST: 1000 us = stop; 1001-2000 us = DShot 48-2047, linear. |
| DS3 | The protocol does not change any state, transition or timing rule of sections 1-14. |
| DS4 | **Open (owner decision, TODO area "reset behaviour"):** after a satellite reset while armed (~720 ms without frames, A4) the ESC loses the signal. If BLHeli_32 needs stop frames before it accepts throttle again, the current FIRST_FRAME_ARMED path (immediate armed throttle) may leave the motors stopped until the throttle goes to zero. Measure on the bench first (B7); then decide whether the board sends `MOTOR_STOP` for a fixed time after boot before passing armed throttle. The same question applies to PWM today. |

## 5. Bench checks (owner + orchestrator; props off unless stated)

| # | Check | Pass |
|---|---|---|
| B1 | Build | Flash and RAM within budget; report the numbers |
| B2 | SWD static, no ESC | TIM1 PSC = 11, ARR = 19, CC1-CC3 enabled, MOE = 1; DMA1 Ch2/Ch3/Ch6 configured; UART1 frames still parse (0 errors, auxFrames ~200/s) |
| B3 | Pin timing (logic analyser, if available) | Bit 3.33 us, T0H ~1.25 us, T1H ~2.5 us, 16 bits per frame, frames at 200 Hz, checksum valid, stop frames while disarmed, 3 motors |
| B4 | ESC detects DShot | Power-up beeps as in the manual's "Normal operation" table, no calibration sequence |
| B5 | INAV configurator motor test | Each motor alone at 1050 / 1200 / 1500: the right motor (M1 O4, M2 O6, M3 O5), stops on release and on the 200 ms timeout |
| B6 | Armed via EVIS | Throttle sweep on all 3; disarm = stop; FW profile: rear motor stopped; FW->MC rear-motor step 1000 -> ~1450 starts cleanly (repeat 10x) |
| B7 | Link pull, satellite reset while armed (combined test_05, A4) | Failsafe stop <= 0.6 s; after link return or reset: **record** how long the ESC takes to run again with throttle above zero (input for DS4) |
| B8 | Ground run, props on, restrained | 5 min, throttle steps 0-100 %, no stalls, no glitches |
| B9 | First flight (owner) | Recheck `throttle_idle` and `nav_mc_hover_thr` (~1420 with PWM): the ESC PWM calibration no longer applies |

## 6. Tests and tools to update

- SITL (`test/run_tests.py`): no change expected (identity conversion). A full run proves it.
- Bench tools that read the motor pulse from TIM1 CCR (`test/tools/hw_output_check.py`, `test/hw/test_hw_01_outputs.py`,
  `test/hw/common/sat_bench.py`, INAV `src/test/adum/hw/hwcommon/satellite_swd.py` MOTOR_PINS and its users: grep
`MOTOR_PINS`, `motor_pulses_us`) are wrong with DShot (the CCR holds bit
  widths). Replace the motor value check with a non-halting SWD read of the commanded value (`motor[]` for us, and the
  DShot value from `dmaMotors[i].protocolControl.value`) plus the static TIM1/DMA checks of B2. Pin-level checks only with
  the logic analyser.
- Texts that say "standard PWM 400 Hz": `README.md`, `test/servo_config_board.txt`, `test/servo_config.txt:33-34`
  ("DShot is not available on the F103"), `STATUS.md`; INAV `docs/adum/tn18_aircraft.md` (motor protocol row),
  INAV `docs/adum/TODO.md` (ESC-RATE), INAV `STATUS.md`.

## 7. Risks

| Risk | Mitigation |
|---|---|
| F1 build of the F3 path never ran anywhere | B2/B3 before any ESC; step 1 compiles with PWM still selected, so the driver exists before it is used |
| Word/byte DMA buffer mismatch, wrong GPIO mode on F1 | Named in 3.1; B3 shows it at once |
| DMA channel conflict with UART1 | No burst mode; B2 checks link errors stay 0 under DShot |
| ESC re-arm after signal loss (DS4) | B7 measures it; owner decides before flight |
| Idle/hover shift after leaving PWM calibration | B9 |
| Long wires to the wing ESCs | Checksum drops bad frames; DShot150 fallback (DS-D2) |

## 8. Agent plan

Rules: INAV `CLAUDE.md` "Agent sessions" (one sub-task per agent, <= ~50k tokens, <= ~30 tool calls, report <= ~300 words,
never continue a finished agent) and this repo's `CLAUDE.md` (firmware agent owns `src/main` except `target/SITL`; test
agent owns `test/` and `target/SITL/target.c`; never two SITL runs at once). Each step is its own commit. The
orchestrator reviews every report before starting the next step. Flashing and bench steps run with the owner.

| Step | Model | Task | Read only | Output / checks |
|---|---|---|---|---|
| 1 | sonnet | Build the F3 DShot driver for F1 (section 3.1). **PWM stays selected** in `config.c`: no behaviour change | This file sections 2, 3.1; `drivers/pwm_output_dshot.c`, `pwm_output_dshot_shared.c`, `dshot_dpwm.h`; `make/mcu/STM32F1.mk`, `STM32F3.mk`; `target/SATELLITEBOARD/target.h`; `pwm_output.c:225-245` for the F1 GPIO pattern. Not: bitbang, HAL files, other targets | Firmware build OK (command in `CLAUDE.md`), flash/RAM numbers vs 68.6 %; SITL build OK; list of every changed `#if`; the four "confirm in code" items answered |
| 2 | sonnet | Protocol-neutral motor values (section 3.2) | This file 3.2; `io/servo_driver.c` `servoDriverWriteMotors()` only; `drivers/motor.c:155-170`; `drivers/dshot.c:60-85` | Firmware + SITL build; `test_05` and `test_07` run once |
| 3 | haiku | SITL regression | `test/README.md` run section | `python3 test/run_tests.py --build`, summary table only; expect all pass as before |
| 4 | sonnet | Switch to DShot300 (section 3.3) + texts in this repo (section 6, last item, BF files only) | This file 3.3, 6; `target/SATELLITEBOARD/config.c` | Firmware build; hex name for flashing |
| 5 | sonnet | Bench tools for DShot (section 6, BF `test/` side) | This file 5 (B2), 6; `test/tools/hw_output_check.py`, `test/hw/test_hw_01_outputs.py`, `test/hw/common/` swd helpers only | Compile-check; dry run against the board if connected. INAV-side bench tools = separate agent in the INAV repo |
| 6 | owner + orchestrator | Flash (`tools/stlink-flash/README.md`), B2-B7 | | Results into `docs/hw_bench_report.md` "DShot"; DS4 data to the owner |
| 7 | orchestrator | Spec section 15 (section 4), DS4 decision recorded; INAV docs (tn18 sheet, TODO, STATUS) | | Docs commit in each repo |
| 8 | owner | B8 ground run, B9 first flight | | |

Step 1 is the only step that touches upstream files. If it needs more than adding F1 next to F3 (new logic, a change for
other MCUs), the agent stops and reports instead of writing it.

## 9. Open questions for the owner

1. DS4: after the B7 measurement, should the board hold `MOTOR_STOP` for a fixed time after a reset before passing armed throttle?
2. Is a logic analyser available for B3 (see INAV `docs/adum/oneshot_bench.md`)? Without it, B3 is skipped and B4-B6 (the ESC itself) are the proof.
3. ESC firmware version: assumed fine for plain DShot (all BLHeli_32 versions support it). Only bidirectional DShot would need >= 32.7.
