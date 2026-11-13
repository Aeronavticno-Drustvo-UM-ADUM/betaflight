# Betaflight, ADUM fork: SATELLITEBOARD servo driver

Wing servo and motor driver of [ADUM](https://adum.si/), the aeronautical research team of the University of Maribor,
for **EVIS**, a VTOL with three tilt rotors and a high wing, built for the SAE Aero Design 2027 competition (Advanced Class).

Fork of [Betaflight](https://github.com/betaflight/betaflight) 4.3.x (MSP API 1.44, based on a 2022 snapshot, not rebased onto current upstream). It adds one target, `SATELLITEBOARD`, a STM32F103 (128 KB flash, 20 KB RAM) board used as a servo and motor driver. It does not fly anything: no gyro, no PID loop, no receiver. The outputs are driven over MAVLink by the master flight controller.

## System

The master flight controller is INAV on EVIS_FC in the fuselage (sibling repo `../inav`, branch `adum-master10+sat`). The SATELLITEBOARD sits in the wing, so only power and one serial bus wire run between the fuselage and the wing.

```
 EVIS_FC, fuselage (INAV, ../inav)          SATELLITEBOARD, wing (this repo)
 master: sensors, navigation, mixer  ---->  5 servos, 3 motors (OneShot125)
                                    UART1   MAVLink 2, ADUM dialect, 50 Hz ADUM_AUX_CHANNELS
                                    <----   HEARTBEAT (ready to arm), STATUSTEXT, SYS_STATUS, ADUM_GPS_PVT
```

The rule (D8, spec [`docs/arming_safety.md`](docs/arming_safety.md)): the master is primary. Master armed = satellite armed; the satellite never refuses an ARMED frame. Only total link loss (no valid frame for 500 ms) stops the motors (FAILSAFE), and the satellite follows the master again as soon as frames return. The satellite reports whether it is ready to arm; INAV blocks its own arming until it is.

Status (2026-11-13): arming, failsafe and safety logic implemented. SITL suite passes (`test/run_tests.py`). On the bench, the standalone suite `test/hw/` (hw_01-04) and the combined suites with EVIS_FC in the INAV repo pass; failsafe delay 499-505 ms. Not yet tested in the aircraft (real ESCs, servos, power). Current state: [`STATUS.md`](STATUS.md); path to the first flight: `../inav/docs/adum/flight_readiness.md`.

Branch: `master+adum+sat`. Rules for agents: [`CLAUDE.md`](CLAUDE.md).

## Hardware (`src/main/target/SATELLITEBOARD/`)

| Function | Pins |
|---|---|
| Servos 1-5 | PA15, PB3 (TIM2, partial remap 1), PA8, PA9, PA10 (TIM1) |
| Motors 1-3 | PA6, PB0, PB1 (TIM3). PA7 is left unused so the motor timer is not shared |
| UART1 / UART2 / UART3 | PB7 RX, PB6 TX (link to the master, remapped) / PA3 RX, PA2 TX (GPS connector) / PB11 RX, PB10 TX (free) |
| USB virtual COM port | for the configurator |
| LED | PC15 |

Pin remaps (UART1, TIM2) are set in `targetPreInit()` in `target.c`, which also switches SWD back on. Config for the real board: `test/servo_config_board.txt` (MAVLink on UART1, GPS on UART2); `test/servo_config.txt` is the SITL variant (MAVLink on UART2). Motors are the TIM3 pins with DMA (PA6, PB0, PB1) for later DShot.

## How it works

- `USE_SERVO_DRIVER` (set in `target.h`) removes the gyro, filter, PID, accelerometer, attitude and RX tasks and the matching init in `main()`. They are replaced by one task (200 Hz target rate; about 150 Hz on SITL), `src/main/io/servo_driver.c`.
- The master flight controller talks MAVLink 2.0 with the ADUM dialect (`lib/main/MAVLink_adum`, generated from `adum.xml` in the INAV fork; MAVLink 1.0 headers stay for the other targets). Put the **MAVLink** telemetry function on a UART (Ports tab or CLI `serial`, 115200 baud). `test/servo_config.txt` is a ready CLI configuration, GPS included.
- **`ADUM_AUX_CHANNELS`** (id 13002) from the master: `arming_status`, `servo[8]`, `motor[3]` in us. The board outputs servos 1-5 and motors 1-3; servos 6-8 are ignored. 0 and 65535 mean "no value". sysid/compid are not checked yet.
- **Servos** always move to the pulse that arrives, armed or not. The configurator servo limits are ignored; only a 500-2500 us clamp applies. Before the first frame they sit at `servo_center_pulse`.
- **Motors** stay at zero throttle (`min_command` pulse) unless the board is ARMED. The motor values of an armed frame are applied at once; a disarmed frame sets the motors to zero. Values are clamped to `min_command`..`max_throttle`. The motor output is written on every task cycle (200 Hz target) in every state, so the ESCs always see a continuous pulse train.
- **Arming** is controlled by `ADUM_AUX_CHANNELS` only; the master is primary (D8). Every ARMED frame arms, whatever the readiness. Readiness (own-health checks: `LINK`, `BOOT_GRACE`, `MOTOR_CONFIG`, `CONFIG_ACTIVE`) is only reported: HEARTBEAT `STANDBY` = ready, `UNINIT` = not ready, plus a 1 Hz `STATUSTEXT` `NOT READY: <reason>` while disarmed and not ready.
- **Failsafe:** armed and no valid `ADUM_AUX_CHANNELS` for `sd_link_timeout_ms` (default 500 ms; `HEARTBEAT` does not count): motors to `min_command`, servos hold the last position. FAILSAFE never times out into DISARMED; it ends with the next frame from the master (armed frame re-arms at once). Silence of 1 s resets the parser; of 2 s re-initialises the UART (not on SITL).
- **Config lock:** while ARMED or FAILSAFE the CLI (enter, `save`, `reboot`, `defaults`, `motor`) and MSP (`MSP_EEPROM_WRITE`, `MSP_SET_MOTOR`, `MSP_RESET_CONF`, `MSP_REBOOT`) are refused. `ARMING_FLAG(ARMED)` stays set in FAILSAFE.
- **Watchdog:** IWDG, 250 ms nominal (LSI +-50 %), STM32F10X only; fed each task cycle and inside the config erase/program loop.
- **CLI settings** (`servoDriverConfig`, `PG_SERVO_DRIVER_CONFIG` 557): `sd_link_timeout_ms` (500, 100-5000), `sd_boot_grace_ms` (2000, 0-10000).
- **Board to master:** a 1 Hz `HEARTBEAT` (DISARMED: base_mode 0 / STANDBY; ARMED: `SAFETY_ARMED` / ACTIVE; FAILSAFE: 0 / CRITICAL), a 1 Hz `SYS_STATUS` (`errors_comm`, `drop_rate_comm` from sequence gaps, `errors_count1` TX drops, `errors_count2` clamps and no-value values; uint16 wrap), `STATUSTEXT` (`NOT READY: <reason>` WARNING, 1 Hz while not ready; `FAILSAFE: link lost` / `link restored` CRITICAL; `LINK: low frame rate` WARNING when < 20 AUX frames in 1 s, max every 10 s), and an **`ADUM_GPS_PVT`** (id 13000) for every UBX NAV-PVT solution (fix type incl. DGPS/RTK float/RTK fixed, time, position, velocity, accuracies). Heading is flagged valid with a 3D fix above 0.3 m/s. `ADUM_MAGNETOMETER` (13001) is not implemented yet (needs a magnetometer driver; I2C1 on the free pins PB8/PB9 needs the AFIO remap that the F1 I2C driver does not set yet).
- GPS (UBlox, set the UART in Ports): the driver configures M9/M10 modules with `UBX-CFG-VALSET` (module detected through `UBX-MON-VER`; SBAS and constellation settings stay at the module defaults on those). Older modules use the original commands.
- Motors are PWM, OneShot or Multishot. **DShot is not available** on this F1 target. The motor protocol defaults to `DISABLED`: set `motor_pwm_protocol` (e.g. `ONESHOT125`) in the CLI.
- Everything else is removed to fit: OSD, blackbox, LED strip, VTX, SPI, beeper, most serial RX protocols and telemetry, resource remapping, plus the unused Open Location Code library (`make/source.mk`). Flash use is 87344 B (text+data) of 126 KB, static RAM 15460 B of 20 KB (2 KB of it is the reserved stack).
- The CLI and MSP over USB/UART still work. The gyro, attitude and PID tabs of the configurator show nothing useful.

## Build

No local ARM toolchain is needed: the INAV build image from the sibling repo (`../inav/Dockerfile`, image `inav-build`) contains `arm-none-eabi-gcc` 10.3.1, host `gcc` 11.4, `make` and `python3` (Ubuntu 22.04).

Create the image once (skip if `docker images -q inav-build` prints an ID; INAV's `./build.sh` creates it too):

```bash
docker build -t inav-build --build-arg USER_ID="$(id -u)" --build-arg GROUP_ID="$(id -g)" ../inav
```

The image is built with your UID/GID, so files in `obj/` stay owned by you. Its default entrypoint is INAV's cmake/ninja script, so every command below replaces it with `--entrypoint`. This Makefile expects arm-none-eabi-gcc 9.3.1 and stops on any other version; `GCC_REQUIRED_VERSION=10.3.1` accepts the one in the image.

Firmware (SATELLITEBOARD), run in this directory:

```bash
docker run --rm --entrypoint make -v "$PWD":/src -w /src inav-build \
    TARGET=SATELLITEBOARD GCC_REQUIRED_VERSION=10.3.1 -j"$(nproc)"
# -> obj/betaflight_4.3.2_SATELLITEBOARD_<commit>.hex
```

Clean: same command with `clean` added (or `rm -rf obj/main/SATELLITEBOARD`).

### SITL

SITL runs on the host and is built with the host `gcc`; it needs only `gcc` and `make`, no ARM toolchain and no Docker. The Makefile checks for arm-none-eabi-gcc even for SITL; giving `GCC_VERSION` and `GCC_REQUIRED_VERSION` the same dummy value skips that check. The test scripts do this themselves:

```bash
test/tools/run_sitl.sh --build        # stock SITL for the configurator, tcp://127.0.0.1:5761 (see "Run SITL")
python3 test/run_tests.py --build     # servo driver SITL (USE_SERVO_DRIVER) + all SITL tests, about 5 min
python3 test/run_tests.py 05          # only files matching "05"
```

The SITL tcp ports 5761-5763 are shared: never run two test runs (or a test run and `run_sitl.sh`) at once. Known SITL startup race (TODO K): about 1 in 3-4 starts crash or lose a listener; the harness detects it and restarts.

Manual build:

```bash
# stock SITL -> obj/main/betaflight_SITL.elf
make TARGET=SITL GCC_VERSION=none GCC_REQUIRED_VERSION=none -j"$(nproc)"

# servo driver SITL -> obj/main_servotest/betaflight_SITL.elf
make TARGET=SITL GCC_VERSION=none GCC_REQUIRED_VERSION=none \
    OBJECT_DIR="$PWD/obj/main_servotest" EXTRA_FLAGS=-DUSE_SERVO_DRIVER -j"$(nproc)"
```

#### Run SITL

`test/tools/run_sitl.sh` builds the binary if missing, starts it in the foreground and stops it with Ctrl-C:

```bash
test/tools/run_sitl.sh                      # stock SITL, config in test/sitl-run/eeprom.bin
test/tools/run_sitl.sh --servo --config     # servo driver SITL; first run: apply test/servo_config.txt
test/tools/run_sitl.sh --servo              # later runs: the config stays in test/sitl-run-servo/eeprom.bin
# --build rebuilds first, --reset deletes eeprom.bin first, --verbose keeps the [FLASH_*] lines, --help
```

| tcp port | UART | use (servo driver with `servo_config.txt`) |
|---|---|---|
| 5761 | UART1 | CLI / MSP: configurator "Manual Selection" `tcp://127.0.0.1:5761`, or `nc 127.0.0.1 5761`, then `#` |
| 5762 | UART2 | MAVLink (ADUM dialect) in and out |
| 5763 | UART3 | UBlox GPS input |

The servo driver SITL prints every output change as `[servo]<index>=<us>` and `[motor]<index>=<us>` on stdout. In SITL, `save` in the CLI exits the process (no reboot); start it again. Without `--config` and without a saved config, UART2 is not MAVLink and the outputs never move.

Drive it with MAVLink: `send_aux_channels.py` opens only serial devices, so bridge a pseudo-terminal to tcp 5762 with `socat` (second terminal):

```bash
socat pty,link=/tmp/sitl-mavlink,raw,echo=0 tcp:127.0.0.1:5762 &
python3 test/tools/send_aux_channels.py /tmp/sitl-mavlink              # servo sweep, disarmed
python3 test/tools/send_aux_channels.py /tmp/sitl-mavlink --arm --motors 1200,1300,1400
```

The script prints the board's HEARTBEAT (arm state) once per second. Stopping it for more than 500 ms puts an armed board into failsafe (motors to `min_command`).

Do not build SITL in the Docker image and run it on the host: such a binary aborted with `malloc(): unaligned fastbin chunk detected` on a Fedora 43 host (it passes when also run inside the container).

## Flash

```bash
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
- `src/main/cli/cli.c`, `src/main/cli/settings.c`: config lock while armed/failsafe, `sd_*` settings.
- `src/main/msp/msp.c`: config lock (MSP error while armed/failsafe).
- `src/main/pg/pg_ids.h`: `PG_SERVO_DRIVER_CONFIG` 557.
- `src/main/config/config_eeprom.c`, `src/main/config/config_streamer.c`: IWDG feed in the erase/program loop.
- `src/main/target/SITL/target.c`: test hooks `[servo]`, `[motor]`, `[motorrate]`; dyad update timeout 0.001 s under `USE_SERVO_DRIVER`.
- `make/source.mk`: `olc.c` is not built for this target. `.gitignore`: tracks `tools/stlink-flash/`.
- Docs: `docs/arming_safety.md` (spec), `TODO.md` (open issues).

## Tests

| Suite | Run | Needs |
|---|---|---|
| SITL | `python3 test/run_tests.py [nn] [--build]` (~5 min) | host only; see "SITL" above |
| Hardware standalone | `python3 test/hw/run_hw_tests.py [NAME] [--repeat N] [--props-off]` | FTDI on the satellite UART1 as fake master, ST-Link (SWD ground truth); props off for tests that arm |
| Combined with EVIS_FC | `../inav/src/test/adum/hw/run_hw_tests.py` | EVIS_FC wired to the satellite |

Every run writes `runs/<time>_<test>/summary.json`. Details: [`test/README.md`](test/README.md), [`test/hw/README.md`](test/hw/README.md).

## Not done yet

Open design decisions and safety risks are listed in [`TODO.md`](TODO.md) and, across both repos, in `../inav/docs/adum/TODO.md`. They need an owner decision before anyone changes that behaviour.

- DShot, servo/motor output feedback, the magnetometer (`ADUM_MAGNETOMETER`), sysid/compid.
- Aircraft tests: real ESCs and servos, power and noise, endurance (`../inav/docs/adum/flight_readiness.md`, parts A and B).
- Bench items still open: watchdog (IWDG) under a firmware hang, reset while armed, pin-level output checks.

Betaflight is licensed under the GPLv3. See the upstream project for general documentation: https://github.com/betaflight/betaflight
