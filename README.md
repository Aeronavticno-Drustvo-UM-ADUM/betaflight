# Betaflight, ADUM fork: SATELLITEBOARD servo driver

Fork of [Betaflight](https://github.com/betaflight/betaflight) 4.3.x (MSP API 1.44, based on a 2022 snapshot, not rebased onto current upstream). It adds one target, `SATELLITEBOARD`, a STM32F103 (128 KB flash, 20 KB RAM) board used as a servo and motor driver. It does not fly anything: no gyro, no PID loop, no receiver. The outputs are driven over MAVLink.

Status: builds and flashes. The driver logic is tested on SITL (`test/test_servo_sitl.py`); not yet tested on the bench with real MAVLink traffic, servos or ESCs.

## Hardware (`src/main/target/SATELLITEBOARD/`)

| Function | Pins |
|---|---|
| Servos 1-5 | PA15, PB3 (TIM2), PA8, PA9, PA10 (TIM1) |
| Motors 1-3 | PA6, PB0, PB1 (TIM3). PA7 is left unused so the motor timer is not shared |
| UART1 / UART2 / UART3 | PB7 RX, PB6 TX / PA3 RX, PA2 TX / PB11 RX, PB10 TX |
| USB virtual COM port | for the configurator |
| LED | PC15 |

## How it works

- `USE_SERVO_DRIVER` (set in `target.h`) removes the gyro, filter, PID, accelerometer, attitude and RX tasks and the matching init in `main()`. They are replaced by one 200 Hz task, `src/main/io/servo_driver.c`.
- The master flight controller talks MAVLink 2.0 with the ADUM dialect (`lib/main/MAVLink_adum`, generated from `adum.xml` in the INAV fork; MAVLink 1.0 headers stay for the other targets). Put the **MAVLink** telemetry function on a UART (Ports tab or CLI `serial`, 115200 baud). `test/servo_config.txt` is a ready CLI configuration, GPS included.
- **`ADUM_AUX_CHANNELS`** (id 13002) from the master: `arming_status`, `servo[8]`, `motor[3]` in us. The board outputs servos 1-5 and motors 1-3; servos 6-8 are ignored. 0 and 65535 mean "no value". sysid/compid are not checked yet.
- **Servos** always move to the pulse that arrives, armed or not. The configurator servo limits are ignored; only a 500-2500 us clamp applies. Before the first frame they sit at `servo_center_pulse`.
- **Motors** stay at zero throttle (`min_command` pulse) unless the last frame said ARMED. The motor values of an armed frame are applied at once (arming status and values arrive in the same frame); a disarmed frame sets the motors to zero. Values are clamped to `min_command`..`max_throttle`.
- **Lost communication:** armed and no valid MAVLink frame (`ADUM_AUX_CHANNELS` or `HEARTBEAT`) for 500 ms disarms the board. The motors go to zero, the servos stay where the last frame put them.
- **Board to master:** a 1 Hz `HEARTBEAT` with the arm state, and an **`ADUM_GPS_PVT`** (id 13000) for every UBX NAV-PVT solution (fix type incl. DGPS/RTK float/RTK fixed, time, position, velocity, accuracies). Heading is flagged valid with a 3D fix above 0.3 m/s. `ADUM_MAGNETOMETER` (13001) is not implemented yet (needs a magnetometer driver; I2C1 on the free pins PB8/PB9 needs the AFIO remap that the F1 I2C driver does not set yet).
- GPS (UBlox, set the UART in Ports): the driver configures M9/M10 modules with `UBX-CFG-VALSET` (module detected through `UBX-MON-VER`; SBAS and constellation settings stay at the module defaults on those). Older modules use the original commands.
- Motors are PWM, OneShot or Multishot. **DShot is not available** on this F1 target. The motor protocol defaults to `DISABLED`: set `motor_pwm_protocol` (e.g. `ONESHOT125`) in the CLI.
- GPS is enabled (UBlox, UART of your choice via Ports). The UBlox driver was extended to configure M9/M10 modules with `UBX-CFG-VALSET` (module detected through `UBX-MON-VER`); SBAS and constellation settings are left at the module defaults on those. Older modules use the original commands.
- Everything else is removed to fit: OSD, blackbox, LED strip, VTX, SPI, beeper, most serial RX protocols and telemetry, resource remapping, plus the unused Open Location Code library (`make/source.mk`). Flash use is about 85 KB of 126 KB, RAM about 15.4 KB of 20 KB (2 KB of it is the reserved stack).
- The CLI and MSP over USB/UART still work. The gyro, attitude and PID tabs of the configurator show nothing useful.

## Build and flash

```bash
make TARGET=SATELLITEBOARD -j$(nproc)
tools/stlink-flash/setup_env.sh        # once
tools/stlink-flash/fix_permissions.sh  # once, needs sudo (udev rule for the ST-Link)
tools/stlink-flash/flash.sh            # flashes over SWD, see tools/stlink-flash/README.md
```

Use an ST-Link on the SWD header. The script detects the chip (medium density STM32F103, pyocd target `stm32f103cb`), logs to `tools/stlink-flash/logs/` and prints hints on failure.

After the first flash, open the CLI and run `defaults` then `save`. Pin layouts are stored in the saved config, so a board that ran another firmware keeps its old pins until then.

## Configurator

Use Betaflight Configurator **10.9.0** ([release](https://github.com/betaflight/betaflight-configurator/releases/tag/10.9.0)), which matches the 4.3 firmware generation. 2025.12.2 also keeps API 1.44 as its minimum. The 2026.x releases were not confirmed to support it.

## Changes against upstream

- New: `src/main/target/SATELLITEBOARD/`, `src/main/io/servo_driver.[ch]`, `src/main/io/adum_mavlink.h`, `lib/main/MAVLink_adum/`, `tools/stlink-flash/`, `test/`.
- `src/main/fc/init.c`, `src/main/fc/tasks.c`, `src/main/scheduler/scheduler.h`: `USE_SERVO_DRIVER` paths.
- `src/main/io/gps.[ch]`: M9/M10 UBlox configuration, NAV-PVT export for `ADUM_GPS_PVT`.
- `make/source.mk`: `olc.c` is not built for this target. `.gitignore`: tracks `tools/stlink-flash/`.

## Not done yet

- DShot, servo/motor output feedback, the magnetometer (`ADUM_MAGNETOMETER`), sysid/compid.
- RAM reduction beyond what is described (about 4 KB free including the stack).
- Bench tests of the real timer outputs, the F103 UART path, GPS M10 and the ESCs.

Betaflight is licensed under the GPLv3. See the upstream project for general documentation: https://github.com/betaflight/betaflight
