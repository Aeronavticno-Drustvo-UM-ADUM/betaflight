# Status

Level: L1/L2 SITL complete for phase 1 (tests 01-05); firmware implementation of `docs/arming_safety.md` done 2026-11-11. Bench started 2026-11-12 (L3 in progress): board flashed and talking MAVLink; first automated HW output check `test/tools/hw_output_check.py` (SWD timer-register reads): servos and disarmed motors PASS, **arm edge fails on hardware (TODO T)**. Report: `docs/hw_bench_report.md`.

## Last results (2026-11-11)
| Item | Result |
|---|---|
| `test/run_tests.py` | 286 checks, 0 failed, 1 known (signed frames accepted), ~5 min; 2 consecutive full runs |
| Failsafe time | 505-516 ms (default 500), 1003 ms at 1000 ms setting |
| Frame -> output latency | median ~3.5 ms, max < 10 ms |
| STATUSTEXT latency | 0-9 ms |
| Parser recovery after truncated frame | N <= 2 frames |
| SATELLITEBOARD size | flash 87344 B of 126 KB, RAM 15460 B of 20 KB |

## Bench (2026-11-12)
Setup, details also in the INAV repo memory/STATUS:
- Board: SATELLITEBOARD PCB, flashed by ST-Link (STM32CubeProgrammer). VCP (CLI/MSP): `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_6D8E568C5670-if00`.
- MAVLink: UART1 (PB6/PB7, 115200) via WCH USB quad serial, channel D: `/dev/serial/by-id/usb-wch.cn_USB_Quad_Serial_0123456789-if06`. Use by-id paths, `/dev/ttyACM*` numbers change on replug.
- Config for the PCB: `test/servo_config_board.txt` (MAVLink UART1, GPS UART2, UART3 off); `test/servo_config.txt` stays the SITL variant.
- Firmware fixes found on the bench (TODO S): USART1 remap (UART1 on PB6/PB7), TIM2 partial remap 1 (PA15/PB3), SWD re-enabled after the remaps. All in `targetPreInit()` (`target.c`), no upstream driver change.
- Verified: HEARTBEAT/SYS_STATUS received, servo outputs S4-S8 (PA8, PA9, PA10, PA15, PB3) 50 Hz on the scope. Clock 72 MHz (8 MHz crystal). The first "no pulses" was a PCB power problem.
- Not verified yet: motors S1/S3/S9 (PA6/PB0/PB1, TIM3, OneShot125; PA7 unused), arming/failsafe on hardware, GPS on UART2, IWDG, `save` vs watchdog, UART re-init.
- Tools: `test/tools/send_aux_channels.py --port <by-id D>` (sine on servos, disarmed; `--arm --motors` only with props off). ST-Link + `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c init -c "mdw <addr> <n>" -c shutdown` reads timer CCR, GPIO and AFIO registers of the running board (use `-c`, a `-f` script printed nothing).

## HW output check (2026-11-12, details `docs/hw_bench_report.md`)
- `test/tools/hw_output_check.py --port <by-id D> [--armed --props-off]`: needs ST-Link + UART1 TTL (no USB to the board, no CLI/MSP open). Servos 1-5 exact, mapping 1 PA15, 2 PB3, 3 PA8, 4 PA9, 5 PA10; motors disarmed at 125 us (PA6, PB0, PB1).
- **Open bug (TODO T):** after disarmed frames, ARMED frames do not arm the board (0-15 s, no STATUSTEXT). Armed motor and failsafe steps (3, 4) therefore not run. Root cause not known; SITL not yet compared.
- Script not committed (untracked).

## Next steps
- **First: root cause of TODO T** on hardware (breakpoint/watchpoint on `state`, `prevFrameArmed` in `servoDriverHandleAux`; flash-vs-source check; try SITL with the same frame sequence). Owner decides the fix.
- Then run `hw_output_check.py --armed --props-off` (steps 3, 4), add CCER/GPIO reads (TODO U), commit the script.
- Hardware test backend for `test/common/harness.py` (`--hw`, serial MAVLink + VCP CLI), one sub-task per piece: transport, then log-free checks (HEARTBEAT, SYS_STATUS, STATUSTEXT, arming, failsafe timing), then an OpenOCD helper that reads CCR registers as output check. Scope only for timing (50 Hz, 200 Hz motor, failsafe edges). Motors armed only with props off.
- Bench test plan: IWDG reset/brownout while armed, `save` vs IWDG, 200 Hz motor pulse on scope, ESC on failsafe/recovery, UART noise/cable pull/re-init, P5, GPS M10.
- SITL phase 2: GPS faults, TX buffer saturation, configurator access while armed.
- SITL phase 3: soak, timer wrap, reboot, EEPROM.
- Open owner decisions (`TODO.md`): E default serial config, G TX drop counter, I signed frames, J IWDG vs `bl`, K SITL startup race (mutex in `serial_tcp.c`), M-R from the INAV <-> satellite SITL suite (2026-11-12).
- INAV <-> satellite SITL suite: INAV repo `src/test/adum/sitl/`, report `docs/adum/satellite_test_report.md` (18 tests, 0 firmware spec failures, findings F1-F10).
- Pushed through `06f7c2e43`; later commits local (not pushed).
