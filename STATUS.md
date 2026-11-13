# Status

Level: L1/L2 SITL complete for phase 1 (tests 01-06, 298 checks, 0 failed 2026-11-13); firmware implementation of `docs/arming_safety.md` done 2026-11-11. Bench started 2026-11-12 (L3 in progress): board flashed and talking MAVLink; first automated HW output check `test/tools/hw_output_check.py` (SWD timer-register reads): servos and disarmed motors PASS, arm edge, armed motors and failsafe PASS (H1 was the WCH adapter). Report: `docs/hw_bench_report.md`.

## Arming rule D8 (2026-11-13)
Owner: the master is primary. The board never refuses an ARMED frame; readiness P1-P5 is reported only (HEARTBEAT `STANDBY` ready / `UNINIT` not ready, STATUSTEXT `NOT READY: <code>` at 1 Hz); unknown message ids are ignored. Master requirements for INAV: `docs/arming_safety.md` section 11; message id rules section 12; accepted ground cases section 13.
- SITL: full suite 310 checks, 0 failed, 1 known, 0 restarts (test_05 136, test_06 29).
- Board: D8 verified (see below); EC fix flashed on top (2026-11-13 evening).
- INAV side of D8 done (INAV `d83e80ef9`: arming flag `SATELLITE`); INAV SITL suite updated, 19 tests, 482 checks, 0 failed.

## Board D8 check: done (2026-11-13)
Firmware `2fa92b817` on the board. `hw_output_check.py --armed --props-off` 3/3 PASS (18/18). Readiness on the board: `UNINIT` + `NOT READY: LINK` after reset, `STANDBY` once AUX frames flow, ARMED edge while not ready arms (`edgesNotReady` +1, reason LINK). Details `docs/hw_bench_report.md` "D8 board check". Readiness steps not yet in `hw_output_check.py` (optional, one `sonnet` agent).

## Fix EC (2026-11-13)
Since fix X, CRC failures of unknown-id frames were not counted at all, so `errors_comm` no longer showed garbage/noise (found by INAV SITL tests 13, 17). `34da60a09`: all errors counted in `errors_comm`; only known-id errors are link errors for readiness. SITL test_05 137/0 (new check: 30 unknown-id frames -> `errors_comm` +30, `STANDBY` kept). Flashed on the board (ELF from `34da60a09`), `hw_output_check.py --armed --props-off` PASS (1 run).

## INAV EVIS_FC <-> board: passed (2026-11-13 night)
EVIS UART3 wired to the board's UART1 (FTDI channel B unplugged). Link, readiness (`UNINIT`/`BOOT_GRACE` -> `STANDBY`), INAV arm block, arm/disarm and FAILSAFE on an INAV TX wire pull with INAV staying armed: all as specified. Details `docs/hw_bench_report.md` "INAV master on the board", INAV `STATUS.md`. Open: exact failsafe delay on this link (INAV TODO S4-fs), one unexplained EVIS reboot (EVIS-RST). With EVIS on UART1, `hw_output_check.py` (FTDI as fake master) needs the FTDI back on the connector.

## Hardware test suite `test/hw/` (2026-11-13 night)
Rerunnable satellite standalone suite (FTDI fake master + SWD): `python3 test/hw/run_hw_tests.py [NAME] [--props-off]`, README `test/hw/README.md`. Tests hw_01 outputs, hw_02 readiness (the D8 board-check steps), hw_03 arm/failsafe (exact failsafe delay via `lastAuxUs`, 3 link losses, recovery): **PASS on the bench 2026-11-13** (10/18/31 checks, suite `--repeat 2` all pass; failsafe delay 502.8-504.2 ms). Fixes from the first run: hw_01 pin-mapping check only for distinct values, hw_02 boot wait 2 s (UART1 receives only ~1.0-1.5 s after an SWD reset). hw_04 error frames (EC: garbage, unknown ids, CRC errors, all three while armed) **PASS** 27/27 x2 (2026-11-13); CRC error -> STANDBY again 1.34-1.35 s; garbage without a start byte is skipped by the parser (not counted, info only). The combined INAV + board suites live in INAV `src/test/adum/hw/` and pass (failsafe delay on the INAV link 499.4-504.5 ms).

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
- Open owner decisions (`TODO.md`): E default serial config, G TX drop counter, J IWDG vs `bl`. M, N, O, P, X closed by D8 (2026-11-13); EC fixed (2026-11-13).
- INAV <-> satellite SITL suite: INAV repo `src/test/adum/sitl/`, report `docs/adum/satellite_test_report.md` (findings F1-F10); now 19 tests, 482 checks, 0 failed (2026-11-13).
- Pushed through `06f7c2e43`; later commits local (not pushed).
