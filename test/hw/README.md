# Satellite standalone hardware tests

Bench: FTDI adapter on satellite UART1 = fake master (50 Hz AUX frames, reads HEARTBEAT/STATUSTEXT); ST-Link SWD =
ground truth (diag block, timer CCR pulses). The real master (EVIS) must NOT be on the satellite link; the
preflight detects it (auxFrames rising while the FTDI is silent).

    test/hw/run_hw_tests.py [NAME ...] [--repeat N] [--props-off]

- Tests: `test/hw/test_hw_*.py`, run as `python3 <file> --run-dir <dir>`; NAME = substring filter.
- Run dir: `test/hw/runs/<time>_<test>_r<k>/` (git-ignored) with `output.log` and `summary.json` (same format as the SITL tests).
- Tests that arm set `ARMS = True`; the runner then requires `--props-off` (propellers removed).
- Exit codes: 0 passed, 1 check/file failed, 2 setup impossible.
- `test_hw_01_outputs.py`: servo patterns, disarmed motors at min_command, and a pin-level check per output pin (timer CCER/MOE enabled, GPIO AF push-pull; AFIO_MAPR recorded).
- `test_hw_04_error_frames.py`: garbage, unknown-id and bad-CRC frames, disarmed and armed (arming_safety.md 4, 5, 12).
- `common/sat_bench.py`: `preflight`, `start_master`, `diag`, `motor_pulses_us`, `servo_pulses_us`,
  `failsafe_delay_ms`, `wait_until`. `common/report.py`: `Report`. The hardware code itself lives in `test/tools/`.
- Flash the firmware first (`obj/main/betaflight_SATELLITEBOARD.elf`); the preflight verifies it over SWD.
