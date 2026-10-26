# Tests

- `run_sitl.sh` starts the stock Betaflight SITL for the configurator (`tcp://127.0.0.1:5761`).
- `test_servo_sitl.py` tests the MAVLink servo driver on a SITL build (`-DUSE_SERVO_DRIVER`, built into `obj/main_servotest`):
  it applies `servo_config.txt` through the CLI, restarts, sends `ADUM_AUX_CHANNELS` frames
  and checks that the servo and motor outputs move, are clamped, and fall back to the safe state on link loss.
  A fake UBlox on the GPS UART feeds NAV-PVT frames and the returned `ADUM_GPS_PVT` messages are checked.
  Run `test/test_servo_sitl.py` (add `--build` after changing the firmware, `--verbose` for the CLI and SITL log).
- `mavlink_gen.py` is the MAVLink 2.0 ADUM frame generator and parser (standard library only). `check_mavlink_gen.sh` compares its frames byte for byte with the ones built by the firmware headers (`mavlink_ref.c`).
- `servo_config.txt` is the CLI configuration for the servo driver; paste it into the configurator CLI for the real board too.
- `send_aux_channels.py [PORT]` sends `ADUM_AUX_CHANNELS` to a real board over a serial device (default `/dev/ttyUSB0`,
  115200 baud): a 0.5 Hz sine sweep of servos 1-5 between 1000 and 2000 us at 50 frames/s, disarmed. It prints the
  board's HEARTBEAT and `ADUM_GPS_PVT` replies. `--arm --motors 1100,1100,1100` arms (remove propellers first).
