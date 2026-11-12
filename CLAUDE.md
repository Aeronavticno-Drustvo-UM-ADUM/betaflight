# Project notes for agents

ADUM fork of Betaflight 4.3: SATELLITEBOARD (STM32F103) MAVLink servo/motor driver. Overview: `README.md`. Tests: `test/README.md`.

- **Open design decisions and risks: `TODO.md`.** Before changing firmware behaviour in those areas (motor pulse generation, re-arm after failsafe, watchdog, reset behaviour, default config, arming flags, TX drops), ask the owner for the decision. Do not decide yourself.
- Do not modify upstream Betaflight code beyond the `USE_SERVO_DRIVER` paths without asking.
- SITL builds on the host (`GCC_VERSION=none GCC_REQUIRED_VERSION=none`), firmware in the `inav-build` Docker image (see `README.md`).
- Run tests: `python3 test/run_tests.py [--build]` (~5 min, 286 checks as of 2026-11-11).
- Never run two SITL test runs at once: shared tcp ports 5761-5763.
- Firmware build: `docker run --rm --entrypoint make -v "$PWD":/src -w /src inav-build TARGET=SATELLITEBOARD GCC_REQUIRED_VERSION=10.3.1 -j"$(nproc)"`.
- Known SITL startup race (TODO K): ~1 in 3-4 starts crash or lose a listener; the harness restarts. Do not patch `serial_tcp.c` without owner approval.
- Subagent split that worked: firmware agent owns `src/main` except `target/SITL`; test agent owns `test/` and `src/main/target/SITL/target.c`.
- Code style (owner rule): write code for humans: clear, readable, maintainable; no consolidated machine-optimised one-liners. Brief description per function and on important lines; minimum wording, no large comment blocks, no comments on self-explanatory code.
