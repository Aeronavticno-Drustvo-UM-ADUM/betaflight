# Status

Level: L1/L2 SITL complete for phase 1 (tests 01-05); firmware implementation of `docs/arming_safety.md` done 2026-11-11. Bench not started. No L3 yet.

## Last results (2026-11-11)
| Item | Result |
|---|---|
| `test/run_tests.py` | 286 checks, 0 failed, 1 known (signed frames accepted), ~5 min; 2 consecutive full runs |
| Failsafe time | 505-516 ms (default 500), 1003 ms at 1000 ms setting |
| Frame -> output latency | median ~3.5 ms, max < 10 ms |
| STATUSTEXT latency | 0-9 ms |
| Parser recovery after truncated frame | N <= 2 frames |
| SATELLITEBOARD size | flash 87344 B of 126 KB, RAM 15460 B of 20 KB |

## Next steps
- Bench test plan: IWDG reset/brownout while armed, `save` vs IWDG, 200 Hz motor pulse on scope, ESC on failsafe/recovery, UART noise/cable pull/re-init, P5, GPS M10.
- SITL phase 2: GPS faults, TX buffer saturation, configurator access while armed.
- SITL phase 3: soak, timer wrap, reboot, EEPROM.
- Open owner decisions (`TODO.md`): E default serial config, G TX drop counter, I signed frames, J IWDG vs `bl`, K SITL startup race (mutex in `serial_tcp.c`).
- Committed and pushed through `06f7c2e43`; `run_sitl.sh --servo/--config` committed 2026-11-12.
