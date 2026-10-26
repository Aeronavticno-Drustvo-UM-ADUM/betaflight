# Tests

- `run_sitl.sh` starts the stock Betaflight SITL for the configurator (`tcp://127.0.0.1:5761`).
- `test_servo_sitl.py` tests the MAVLink servo driver on a SITL build (`-DUSE_SERVO_DRIVER`, built into `obj/main_servotest`):
  it applies `servo_config.txt` through the CLI, restarts, sends arm and `RC_CHANNELS_OVERRIDE` frames
  and checks that the servo and motor outputs move, are clamped, and fall back to the safe state on link loss.
  Run `test/test_servo_sitl.py` (add `--build` after changing the firmware, `--verbose` for the CLI and SITL log).
- `mavlink_gen.py` is the MAVLink v1 frame generator (standard library only); its output was compared byte for byte with the firmware's MAVLink headers.
- `servo_config.txt` is the CLI configuration for the servo driver; paste it into the configurator CLI for the real board too.
