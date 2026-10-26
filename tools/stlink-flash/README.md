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
