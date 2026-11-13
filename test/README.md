# Tests

SITL tests of the SATELLITEBOARD MAVLink servo driver (`src/main/io/servo_driver.c`). They run on the host, no hardware needed.

## Layout

```
test/
  run_tests.py                  runs all test_*.py one after the other, summary table, exit != 0 on failure
  servo_config.txt              CLI configuration of the servo driver (also for the real board)
  common/mavlink_gen.py         MAVLink 2.0 ADUM frame generator and parser (standard library only)
  common/harness.py             Sitl, BoardLink, FakeUblox, config/build helpers, check()/known_issue()
  Sitl.time_jump() / Fixture.time_jump()  SITL only: SIGUSR1 adds 25 min to micros()/millis() per call (below the 35.8 min
                                wrap of the 32-bit microsecond counter, so per-second timers keep working); delays do not sleep longer
  test_01_normal_operation.py   healthy link
  test_02_link_loss.py          failsafe timing and recovery
  test_03_frame_integrity.py    damaged, unusual and foreign frames
  test_04_gps_pvt.py            UBX NAV-PVT -> ADUM_GPS_PVT
  test_05_arming_safety.py      arming/safety rules of docs/arming_safety.md: states, arm edge, own-health checks, config lock, reporting
  tools/run_sitl.sh             stock or servo driver (--servo [--config]) SITL to run by hand (tcp://127.0.0.1:5761)
  tools/send_aux_channels.py    sends ADUM_AUX_CHANNELS to a real board over serial
  tools/check_mavlink_gen.sh    compares mavlink_gen.py byte for byte with the firmware headers (tools/mavlink_ref.c)
```

## Run

```bash
test/run_tests.py --build        # build SITL (-DUSE_SERVO_DRIVER, obj/main_servotest) once, run everything
test/run_tests.py 02             # only files whose name contains "02"
test/test_02_link_loss.py        # one file; options --build --keep --verbose
test/tools/check_mavlink_gen.sh  # generator vs C headers
```

The files share the tcp ports 5761 (CLI), 5762 (MAVLink on UART2), 5763 (GPS on UART3) and therefore run sequentially. **Run one test run at a time**, never two in parallel (and not together with `tools/run_sitl.sh`).
Known SITL startup race (TODO K, SITL tcp layer is not thread-safe): about 1 in 3-4 starts crash (SIGSEGV) or lose a listener; the harness detects this and restarts SITL.
Each file configures a fresh SITL (`servo_config.txt` through the CLI, then restart), SITL runs in real time.
Logs: `test/sitl-run-logs/` (git-ignored). Exit codes: 0 all checks passed, 1 a check failed, 2 setup impossible (ports busy, no binary).
Output lines: `[PASS]`, `[FAIL]`, `[KNOWN]` (current behaviour that is an open design question, does not fail the run).
SITL test hooks (`src/main/target/SITL/target.c`): `[servo]i=v` / `[motor]i=v` on change, `[motorrate]n` = motor-0 writes in the last second.
Not testable on SITL (bench): P5 save pending (save is synchronous), IWDG, ESC behaviour.
Timing checks use `time.monotonic()` of the sender and the timestamps of the `[servo]`/`[motor]` lines SITL prints on every output change; tolerances are chosen for a loaded desktop (failsafe 495-600 ms after the last frame, frame to output < 30 ms).

## Test catalogue and phase plan

| Phase | File | Scenarios |
|---|---|---|
| 1 | test_01_normal_operation | start-up, disarmed frames, arm/disarm, sweep, frame rates 10/50/100 Hz, latency, clamps and no-value markers, arming_status 2/255, arm toggling |
| 1 | test_02_link_loss | gaps below/above the timeout, recovery (disarmed first / armed directly), flapping, HEARTBEAT only, TCP reconnect, 10 s outage |
| 1 | test_03_frame_integrity | bad CRC, garbage, byte by byte, truncated frame, burst, unknown id / MAVLink 1 / wrong CRC_EXTRA, trimmed payload, other sysid/compid, signed frame |
| 1 | test_04_gps_pvt | NAV-PVT conversion (fix types, flags, time, DOP) |
| 1 | test_05_arming_safety | transitions + timing, FAILSAFE 60 s, arm-edge rule (boot / gap), P1-P4 rejections with STATUSTEXT, config lock (CLI/MSP), motor writes every cycle per state (~150 Hz on SITL, 200 Hz bench), HEARTBEAT status, SYS_STATUS, parser reset, sd_* settings |
| 2 | (planned) | GPS faults, TX buffer saturation, configurator access while armed |
| 3 | (planned) | soak, timer wrap, reboot, EEPROM |

Last full run 2026-11-11: 286 checks, 0 failed, 1 known (signed frames accepted), about 5 min.

Open design decisions and risks: see ../TODO.md
