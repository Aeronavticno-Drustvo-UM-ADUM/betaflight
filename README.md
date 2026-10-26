# Betaflight, ADUM fork: SATELLITEBOARD servo driver

Fork of [Betaflight](https://github.com/betaflight/betaflight) 4.3.x (MSP API 1.44, based on a 2022 snapshot, not rebased onto current upstream). It adds one target, `SATELLITEBOARD`, a STM32F103 (128 KB flash, 20 KB RAM) board used as a servo and motor driver. It does not fly anything: no gyro, no PID loop, no receiver. The outputs are driven over MAVLink.

Status: builds and flashes. Not yet tested on the bench with real MAVLink traffic, servos or ESCs.

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
- Send MAVLink on the port that has the **MAVLink** telemetry function (set it in the configurator Ports tab or CLI `serial`; default 57600 baud).
- **`RC_CHANNELS_OVERRIDE`** (target system 0 or 1), pulse width in us: channels 1-5 are servos (500-2500), channels 6-8 are motors (clamped to `mincommand`..`maxthrottle`). 0 and 65535 mean "do not change".
- **Arming:** the `HEARTBEAT` armed flag, or `MAV_CMD_COMPONENT_ARM_DISARM`, together with a recent `RC_CHANNELS_OVERRIDE`. The board also sends a 1 Hz `HEARTBEAT` (sysid 1, comp `MAV_COMP_ID_SERVO1`) with its arm state.
- **Link-loss protection:** no `RC_CHANNELS_OVERRIDE` for 500 ms disarms, centres the servos (`servo_center_pulse`) and sets the motors to `mincommand`. The arm request is cleared, so the ground side must arm again.
- Motors are PWM, OneShot or Multishot. **DShot is not available** on this F1 target. The motor protocol defaults to `DISABLED`: set `motor_pwm_protocol` (e.g. `ONESHOT125`) in the CLI.
- GPS is enabled (UBlox, UART of your choice via Ports). The UBlox driver was extended to configure M9/M10 modules with `UBX-CFG-VALSET` (module detected through `UBX-MON-VER`); SBAS and constellation settings are left at the module defaults on those. Older modules use the original commands.
- Everything else is removed to fit: OSD, blackbox, LED strip, VTX, SPI, beeper, most serial RX protocols and telemetry, resource remapping, plus the unused Open Location Code library (`make/source.mk`). Flash use is about 82 KB of 126 KB, RAM about 16 KB of 20 KB (2 KB of it is the reserved stack).
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

- New: `src/main/target/SATELLITEBOARD/`, `src/main/io/servo_driver.[ch]`, `tools/stlink-flash/`.
- `src/main/fc/init.c`, `src/main/fc/tasks.c`, `src/main/scheduler/scheduler.h`: `USE_SERVO_DRIVER` paths.
- `src/main/io/gps.[ch]`: M9/M10 UBlox configuration.
- `make/source.mk`: `olc.c` is not built for this target. `.gitignore`: tracks `tools/stlink-flash/`.

## Not done yet

- DShot, servo/motor output feedback, MAVLink telemetry other than the heartbeat.
- RAM reduction beyond what is described (about 4 KB free including the stack).
- Bench tests of the arm logic, link-loss disarm, GPS M10 and the motor outputs.

Betaflight is licensed under the GPLv3. See the upstream project for general documentation: https://github.com/betaflight/betaflight
