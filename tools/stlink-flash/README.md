# ST-Link flashing

Flash a built target (default `SATELLITEBOARD`) over the SWD header with an ST-Link.

```bash
tools/stlink-flash/setup_env.sh          # once: creates stlinkenv/ and installs pyocd
tools/stlink-flash/fix_permissions.sh    # once: udev rule so no root is needed (asks for sudo)
make TARGET=SATELLITEBOARD -j$(nproc)    # build
tools/stlink-flash/flash.sh              # flash the newest hex for the target
```

- `flash.sh [TARGET] [--build] [--erase-chip] [--no-reset] [--dry-run]`
- The chip is detected over SWD (medium density 0x410 -> `stm32f103cb`, high density -> `stm32f103rc`).
  The CMSIS pack for a detected chip is downloaded on first use. Override with `PYOCD_TARGET=<name>`.
- Each run is logged to `tools/stlink-flash/logs/` and common failures print a hint
  (no probe, permissions, wrong flash algorithm, read protection, SWD wiring).
- After the first flash run `defaults` then `save` in the CLI: the config saved in flash is only replaced then.

## Troubleshooting (macOS, learned 2026-11-21)

- **Probe hangs while programming:** reads, chip ID, erase and RAM writes work, but every flash write stops after
  ~2 s with `USB Error: [Errno 60] Operation timed out` (pyocd) or `timed out while waiting for target halted` /
  `error waiting for target flash write algorithm` (OpenOCD), for any image, tool and SWD speed. Cause: the STLINK-V3
  plugged directly into the MacBook's USB-C port. Fix: connect it over USB-A through the external dock.
  A failed run leaves the flash partly erased; the board does not boot until a flash completes.
- **STM32CubeProgrammer** (GUI) works as an alternative: full chip erase, verify. Afterwards the core was in LOCKUP
  (DHCSR `0xE000EDF0` = `0x0109xxxx`, all RCC clocks off): reset once, e.g.
  `openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init" -c "reset run" -c "exit"`, or power-cycle.
- `flash.sh` prints `stat: illegal option -- c` on macOS (GNU `stat`); harmless. The `lsusb` permission hint is Linux only.
- **Verify without halting** (core keeps running):
  ```bash
  openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init" \
      -c "verify_image obj/<image>.hex" -c "mdw 0xE000EDF0 1" -c "mdw 0x40012C28 1" -c "mdw 0x40012C2C 1" \
      -c "mdw 0x40012C34 3" -c "exit"
  ```
  Expect `verified <size> bytes` (the CRC shortcut fails on a running core; the byte compare that follows counts),
  DHCSR without bit 19 (lockup), TIM1 PSC `2` / ARR `0xEA5F` (24 MHz, 2.5 ms = 400 Hz motors), CCR1-3 `0x5DC0`
  (1000 us, disarmed).
