# Status

Level: L1/L2 SITL complete for phase 1 (tests 01-06, 298 checks, 0 failed 2026-11-13); firmware implementation of `docs/arming_safety.md` done 2026-11-11. Bench started 2026-11-12 (L3 in progress): board flashed and talking MAVLink; first automated HW output check `test/tools/hw_output_check.py` (SWD timer-register reads): servos and disarmed motors PASS, arm edge, armed motors and failsafe PASS (H1 was the WCH adapter). Report: `docs/hw_bench_report.md`.

## Arming rule D8 (2026-11-13)
Owner: the master is primary. The board never refuses an ARMED frame; readiness P1-P5 is reported only (HEARTBEAT `STANDBY` ready / `UNINIT` not ready, STATUSTEXT `NOT READY: <code>` at 1 Hz); unknown message ids are ignored. Master requirements for INAV: `docs/arming_safety.md` section 11; message id rules section 12; accepted ground cases section 13.
- SITL: full suite 310 checks, 0 failed, 1 known, 0 restarts (test_05 136, test_06 29).
- Board: not flashed with D8 yet (`hw_output_check.py --armed` run pending, needs the owner at the bench).
- INAV SITL suite (`../inav/src/test/adum/sitl/`): tests 12 (arm rejected), 09, 10, 02, 15, 17, 18 still encode the old reject / STANDBY behaviour.

## Next session: board D8 check (owner at the bench, props off)
Board still runs the pre-D8 firmware. Do not run hardware checks while an agent rebuilds the ELF.
1. Build: `docker run --rm --entrypoint make -v "$PWD":/src -w /src inav-build TARGET=SATELLITEBOARD GCC_REQUIRED_VERSION=10.3.1 -j"$(nproc)"`.
2. Flash: `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program obj/main/betaflight_SATELLITEBOARD.elf verify reset exit"`.
3. Run: `python3 test/tools/hw_output_check.py --port /dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0 --armed --props-off` (3 runs; was 18/18 before D8). SWD: `test/tools/read_diag.py` (fields renamed: `edgesNotReady`, `lastNotReadyReason`, `notReadyTexts*`).
4. Not covered by `hw_output_check.py` yet, check on the board: after power-up HEARTBEAT `UNINIT` + `NOT READY: LINK`/`BOOT_GRACE`, then `STANDBY` once AUX frames flow; an ARMED frame during `UNINIT` arms (`edgesNotReady` +1). Either a short manual check or one `sonnet` agent adding a readiness step to `hw_output_check.py` (decide at session start).
5. Record results here and in `docs/hw_bench_report.md`; then INAV D8 is the owner's.

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
- MAVLink: UART1 (PB6/PB7, 115200) via FTDI FT2232H channel B: `/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0`. Do not use the WCH USB quad serial: it delayed board->host data by 4-15 s (false H1). Use by-id paths.
- Config for the PCB: `test/servo_config_board.txt` (MAVLink UART1, GPS UART2, UART3 off); `test/servo_config.txt` stays the SITL variant.
- Firmware fixes found on the bench (TODO S): USART1 remap (UART1 on PB6/PB7), TIM2 partial remap 1 (PA15/PB3), SWD re-enabled after the remaps. All in `targetPreInit()` (`target.c`), no upstream driver change.
- Verified: HEARTBEAT/SYS_STATUS received, servo outputs S4-S8 (PA8, PA9, PA10, PA15, PB3) 50 Hz on the scope. Clock 72 MHz (8 MHz crystal). The first "no pulses" was a PCB power problem.
- Not verified yet: motor timing on the scope, GPS on UART2, IWDG, `save` vs IWDG, UART re-init.
- Tools: `test/tools/send_aux_channels.py --port <FTDI B>` (sine on servos, disarmed; `--arm --motors` only with props off). `test/tools/read_diag.py`: diagnostic counters over SWD (arm state ground truth). ST-Link + `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c init -c "mdw <addr> <n>" -c shutdown` reads registers of the running board.

## HW output check (2026-11-12, updated 2026-11-13; details `docs/hw_bench_report.md`)
- `test/tools/hw_output_check.py --port <FTDI B> [--armed --props-off]`: needs ST-Link + UART1 TTL (no USB to the board, no CLI/MSP open). Servos 1-5 exact, mapping 1 PA15, 2 PB3, 3 PA8, 4 PA9, 5 PA10; motors disarmed at 125 us (PA6, PB0, PB1). Arm/disarm/failsafe also checked over SWD (diag counters).
- **H1 closed (2026-11-13):** not a firmware bug; the WCH adapter delayed the board's HEARTBEAT. With FTDI: 3/3 PASS incl. armed motors, disarm, failsafe on link loss (steps 3, 4).

## Next steps
- **Done 2026-11-13:** D8 (master is primary, `docs/arming_safety.md`); fix W (int32 timestamp windows, `docs/fix_W_timestamps.md`, SITL `test_06_time_wrap.py`, verified on the board); diagnostic counters `USE_SERVO_DRIVER_DIAG` (on for SATELLITEBOARD); K (SITL startup race) fixed with a mutex in `serial_tcp.c` (SITL only). Next: CCER/GPIO reads (TODO U), remaining bench plan (V), owner decisions (TODO.md, X).
- Next (board): CCER/GPIO reads in `hw_output_check.py` (TODO U); remaining bench plan (V) with the owner.
- Hardware test backend for `test/common/harness.py` (`--hw`, serial MAVLink + VCP CLI), one sub-task per piece: transport, then log-free checks (HEARTBEAT, SYS_STATUS, STATUSTEXT, arming, failsafe timing), then an OpenOCD helper that reads CCR registers as output check. Scope only for timing (50 Hz, 200 Hz motor, failsafe edges). Motors armed only with props off.
- Bench test plan: IWDG reset/brownout while armed, `save` vs IWDG, 200 Hz motor pulse on scope, ESC on failsafe/recovery, UART noise/cable pull/re-init, P5, GPS M10.
- SITL phase 2: GPS faults, TX buffer saturation, configurator access while armed.
- SITL phase 3: soak, timer wrap, reboot, EEPROM.
- Open owner decisions (`TODO.md`): E default serial config, G TX drop counter, J IWDG vs `bl`. M, N, O, P, X closed by D8 (2026-11-13).
- INAV <-> satellite SITL suite: INAV repo `src/test/adum/sitl/`, report `docs/adum/satellite_test_report.md` (18 tests, 0 firmware spec failures, findings F1-F10).
- Pushed through `06f7c2e43`; later commits local (not pushed).
