# Status

Branch `adum-master` (created by the owner at `551292487` = `master+adum+sat`; that branch is no longer used). Master side:
INAV `../inav` branch `adum-master10`. Everything up to `551292487` is pushed. Push only on request.
Master list of open items for both repos: INAV `docs/adum/TODO.md`; next steps: INAV `STATUS.md`.

## Level (2026-11-20)
- **Arming rule D8 "the master is primary"** (`docs/arming_safety.md` 11-13): the board never refuses an ARMED frame;
  readiness P1-P5 is reported only (HEARTBEAT `STANDBY` / `UNINIT`, STATUSTEXT `NOT READY: <code>` at 1 Hz). Only total
  link loss (no valid frame for 500 ms) gives FAILSAFE. Unknown message ids are ignored; all errors count in `errors_comm`,
  only known-id errors are link errors.
- **MOTOR_TEST** (section 14, MT1-MT10): `arming_status` 2 while the master is disarmed with its motor test active; the
  board stays DISARMED and drives motors 1-3 (200 ms validity). Ignored in ARMED/FAILSAFE, 5 s lockout after leaving them,
  blocked in boot grace.
- **Pinout (PINOUT-9):** outputs O1-O9 (`docs/adum/sat_pinout.png`): servos S1-S6 on O1 O9 O2 O8 O3 O7 (PA6 PB1 PA7 PB3
  PB0 PA15, TIM3/TIM2), motors M1-M3 on O4 O6 O5 (PA8 PA9 PA10, TIM1 only). O = PCB header, S = servo, M = motor.
- **Defaults only:** the board always boots on the compiled defaults in `target/SATELLITEBOARD/config.c` (MAVLink UART1,
  GPS UART2, standard PWM motors 50 Hz); saved config never loaded (`USE_CONFIG_DEFAULTS_ONLY`). CLI `set`/`save` lasts
  until reboot only.
- **SERVO-1** (INAV `b69045b2d`): AUX slot 0 = INAV servo 1. Board unchanged (slot 0 = S1); dialect text synced (`68a52508b`).
- Dialect `lib/main/MAVLink_adum` is kept identical to INAV `lib/main/MAVLink` (`diff -rq` = only README + INAV generator files).
- SITL: 7 tests, all pass (2026-11-14; test_07 motor test 49/49 x3).
- Board: flashed 2026-11-14 with `551292487` (ST-Link, verified); link with EVIS up. Owner checked all outputs and the
  motor tab. The owner may have flashed since: check before relying on it.
- Bench: `test/hw/` hw_01-04 and `test/tools/hw_output_check.py` updated to the new pins and standard PWM: compile-checked
  only, **not run on the new pinout** (needs FTDI B on satellite UART1). INAV combined suite passes on the new pinout (254/254).
- Not yet tested in the aircraft by tests here; owner's TN18 first flight 2026-11-17 (INAV `STATUS.md`).

## Next steps (board side)
1. On the owner's go, after the FTDI rewire: `test/hw/` hw_01-04 and `hw_output_check.py` on the new pinout; then update
   `docs/hw_bench_report.md`.
2. Bench items still open (TODO V): 50 Hz motor/servo timing on the scope, IWDG reset while armed, `save` vs IWDG, UART
   re-init after silence, P5, GPS M10 on UART2, ESC on failsafe/recovery.
3. Owner decisions (`TODO.md`): G TX drop counter, J IWDG vs `bl`; optional UART1 fallback, settings-write lock (R-S7/R-S8),
   OneShot125/DShot (`PWM_TYPE_STANDARD` in target `config.c`).
4. SITL later: phase 2 (GPS faults, TX buffer saturation, configurator access while armed), phase 3 (soak, reboot, EEPROM).

## Tests
- SITL: `python3 test/run_tests.py [nn] [--build]` (README `test/README.md`). Never two SITL runs at once (ports 5761-5763),
  also not together with the INAV suite.
- Standalone bench suite: `python3 test/hw/run_hw_tests.py [NAME] [--props-off]` (README `test/hw/README.md`): FTDI B as
  fake master + SWD. Last PASS on the old pinout 2026-11-13: hw_01 10, hw_02 18, hw_03 31 (failsafe 502.8-504.2 ms),
  hw_04 27 checks, `--repeat 2`.
- Combined suites with INAV EVIS_FC: INAV `src/test/adum/hw/` (failsafe delay on the INAV link 499.4-504.5 ms).
- History of the bench findings (USART1/TIM2 remaps, WCH adapter delay H1, D8 board check, EC): `docs/hw_bench_report.md`.

## Bench
Setup also in INAV `STATUS.md` (Bench) and memory `satellite-bench-setup`.
- Board VCP (CLI/MSP): `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_6D8E568C5670-if00`.
- MAVLink UART1 (PB6/PB7, 115200): EVIS UART3 in the combined setup, or FTDI FT2232H channel B
  (`/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0`) as fake master. Do not use the WCH quad serial (delays 4-15 s).
- ST-Link + OpenOCD (`-f interface/stlink.cfg -f target/stm32f1x.cfg`), one OpenOCD at a time. Flash:
  `openocd ... -c "program obj/main/betaflight_SATELLITEBOARD.elf verify reset exit"`. Register read of the running board:
  `-c init -c "mdw <addr> <n>" -c shutdown`.
- Tools (`test/tools/`): `read_diag.py` (diag counters over SWD, arm state ground truth), `sat_usb_monitor.py` (USB/MSP:
  state, S1-S6, M1-M3), `sat_monitor.py` (SWD: frames/s, counters), `send_aux_channels.py --pinout/--output O1..O9`
  (FTDI B as fake master; `--arm --motors` only with props off), `hw_output_check.py`.
- Props off for armed steps; the owner wires the boards.
