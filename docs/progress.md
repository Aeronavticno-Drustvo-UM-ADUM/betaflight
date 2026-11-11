# Progress and decisions

## 2026-11-11
Milestones:
- Test suite restructured: `test/run_tests.py`, `test/common/` (harness, generator), `test/tools/`, `test_01`..`test_05`; old `test_servo_sitl.py` removed.
- `docs/arming_safety.md` approved (owner D1-D7) and implemented in `servo_driver.c`, CLI, MSP, config streamer, SITL hooks.
- SITL: 286 checks, 0 failed, 1 known; 2 consecutive full runs. Bench pending.

Decisions (owner) and rationale:
| Decision | Rationale |
|---|---|
| Arming by `ADUM_AUX_CHANNELS` only; HEARTBEAT ignored | One authority, no ambiguous second source |
| Failsafe: no AUX frame for 500 ms (`sd_link_timeout_ms`): motors zero, servos hold | Servos keep the aircraft controllable; motors cut is the only safe unattended action |
| Master primary; own-health checks only on an observed arm edge | Satellite must not duplicate master checks; a wrong disarm in flight loses the aircraft |
| First frame of a link session saying ARMED arms at once, no checks | Covers board reset and link recovery in flight |
| FAILSAFE never times out into DISARMED | No automatic disarm in flight; ends only with a master frame |
| No throttle check on the satellite | Master's job (D3/D5) |
| No `ARMED_FORCE` dialect value | Not needed: recovery is covered by the first-frame rule |
| Motors written every 200 Hz cycle | Continuous ESC pulse train (TODO A) |
| IWDG 250 ms, F103 only, fed in the save loop | Hang protection (TODO C) |
| Config lock while ARMED/FAILSAFE | No configurator interference in flight (TODO F) |

Rejected approaches:
- Pre-arm checks blocking re-arm in flight: would prevent recovery after a link glitch.
- Reset-cause based in-flight restart detection: unnecessary, the first-frame rule covers it.
- FAILSAFE -> DISARMED timeout: would cut the motors of a flying aircraft.

Limitations: see `TODO.md` (E, G, I, J, K, L).
