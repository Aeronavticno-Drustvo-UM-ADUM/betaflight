# SATELLITEBOARD arming and safety specification

Status: **APPROVED and IMPLEMENTED 2026-11-11 (SITL-tested, bench pending).** Owner decisions D1-D7 below.
Scope: `src/main/io/servo_driver.c` and the paths it controls. Open risks: `../TODO.md`. Tests: `../test/README.md`.

## 1. Principles (owner, 2026-11-11)
1. **The master is the primary arming authority.** The satellite does not repeat the master's pre-arm checks: throttle position, sensors and so on are the master's job. The satellite checks only its own health: init done, link health, output configuration, no configurator session. It checks these **only on an arm edge it observes itself** (section 4); a failed check is reported to the master with a reason.
2. **Armed:** the board follows the master. The only event that disarms the motors without a master command is **loss of the link to the master**. Everything else is reported, never acted on by disarming. A wrong disarm in flight means losing the aircraft.
3. **Link loss detection must not give false alarms.** Recovering the link is part of the design.
4. Servos always follow valid commands, in every state. In failsafe they hold their last position.
5. Every rule here has a test in `test/test_05_arming_safety.py` (SITL) or in a bench test (hardware-only items).

## 2. States
| State | Motors | Servos | HEARTBEAT `base_mode` / `system_status` |
|---|---|---|---|
| `BOOT` (init up to the first task cycle) | `min_command` | `servo_center_pulse` | (none sent) |
| `DISARMED` | `min_command` | follow valid frames | 0 / `MAV_STATE_STANDBY` |
| `ARMED` | follow armed frames, clamped | follow valid frames | `SAFETY_ARMED` / `MAV_STATE_ACTIVE` |
| `FAILSAFE` (armed intent, link lost) | `min_command` | hold the last valid value | `SAFETY_ARMED` cleared / `MAV_STATE_CRITICAL` |

The motor output is written **on every task cycle (200 Hz)** in every state, not only when a frame arrives. This fixes TODO A: the ESCs always see a continuous pulse train, including `min_command` in failsafe.

## 3. Transitions
| From | To | Condition | Notes |
|---|---|---|---|
| `BOOT` | `DISARMED` | init done, normal start (power-on / pin reset) | |
| `DISARMED` | `ARMED` | **arm edge** (section 4): armed frame after a disarmed frame of the same link session, **and** all own-health checks pass | motors follow that same frame |
| `DISARMED` | `DISARMED` | arm edge, an own-health check fails | report rejection + reason (section 7). The board stays disarmed while the master keeps sending ARMED: the master must disarm and arm again (no delayed spin-up) |
| `DISARMED` | `ARMED` | **first frame of a link session** says ARMED (board rebooted or link recovered while the master was armed) | immediately, **no checks**: the master is primary |
| `ARMED` | `DISARMED` | valid frame with `arming_status` != ARMED | immediate, by design |
| `ARMED` | `FAILSAFE` | no valid `ADUM_AUX_CHANNELS` for `T_fs` (default 500 ms) | HEARTBEAT and other messages do not count |
| `FAILSAFE` | `ARMED` | valid armed frame (first frame of the new link session) | in-flight recovery: **no checks**, the master decides (resolves TODO B) |
| `FAILSAFE` | `DISARMED` | valid disarmed frame | |

**Link session:** starts with the first valid `ADUM_AUX_CHANNELS` after boot or after a gap > `T_fs`. **D1 (owner): FAILSAFE never times out into DISARMED**; it ends only with a frame from the master.

## 4. Own-health checks (only on an observed arm edge, DISARMED -> ARMED)
These run only when the board itself saw the master go from disarmed to armed in the current link session. They never apply to the first frame of a link session, nor to recovery from FAILSAFE.

| # | Check | Reason code |
|---|---|---|
| P1 | Link healthy: at least 5 consecutive valid AUX frames, no CRC error in the last 1 s | `LINK_NOT_READY` |
| P2 | Init done: time since boot >= `T_boot` (2 s, ESC initialisation) | `BOOT_GRACE` |
| P3 | Motor output usable: `motor_pwm_protocol` != DISABLED, motors enabled | `MOTOR_CONFIG` |
| P4 | No configurator session: CLI not active, no MSP write or motor test in the last 2 s | `CONFIG_ACTIVE` |
| P5 | No config save pending or in progress | `CONFIG_ACTIVE` |

Not checked by the satellite (owner, D3/D5): throttle position, GPS, anything else the master's own pre-arm checks cover.

## 5. While armed: events that never disarm
| Event | Handling |
|---|---|
| Invalid / out-of-range values | Clamp; 0 / 65535 = keep the previous value (servos) or `min_command` (motors). Counted. |
| CRC errors, garbage, short frames | Ignored, counted, reported in the link quality. |
| Configurator / MSP / CLI | **Refused while armed or in failsafe:** entering the CLI, EEPROM save, reboot, `defaults`, motor test, MSP set-motor. MSP read-only stays allowed. |
| GPS lost / corrupt | Only reported (`ADUM_GPS_PVT` stops or shows no fix). |
| TX buffer full | Frame dropped, counter incremented (TODO G). |

## 6. Board reset during flight
- **Watchdog (TODO C):** IWDG enabled on the F103 target. Timeout 250 ms (D4, approved). The LSI clock on the F103 is +-50 %, so the real timeout is 170-500 ms. It is fed from the scheduler. No flash operation runs while armed, because of section 5.
- **In-flight restart:** no reset-cause logic is needed. After any reset (watchdog, brownout, power-on), the first frame of the new link session decides (section 3): ARMED re-arms at once. A master that resets comes up disarmed and stays so until its own pre-arm checks pass (owner, D5).
- **Flash operations vs the watchdog:** the watchdog is fed inside the config save loop (erase/write), so a `save` while disarmed does not cause a reset.
- During boot (about 1-2 s), outputs are servos at the centre and motors at `min_command` (TODO D). This is accepted unless the owner decides otherwise.

## 7. Reporting to the master
Until the ADUM dialect has dedicated messages, standard common messages are used:
- **HEARTBEAT** (1 Hz): state as in section 2. The failsafe check runs before the HEARTBEAT is sent in the same cycle (fixes TODO H).
- **STATUSTEXT**, severity WARNING, text `ARM REJECTED: <reason code>`. Sent on every rejected arm request, at most 2 Hz.
- **STATUSTEXT**, severity CRITICAL, text `FAILSAFE: link lost` / `FAILSAFE: link restored`. Sent on each transition.
- **SYS_STATUS** (1 Hz):
  - `errors_comm` = CRC/parse errors since boot;
  - `drop_rate_comm` = lost AUX frames over the last 1 s, from MAVLink sequence gaps, in 0.01 %;
  - `errors_count1` = TX drops; `errors_count2` = clamped values.
- Later, a dialect message `ADUM_STATUS` (state, reject reason, counters) is planned; the master code will need changes for it.

## 8. Link loss detection and recovery
- **Detection:** only a valid `ADUM_AUX_CHANNELS` (CRC OK, any sysid) refreshes the timer. `T_fs` defaults to 500 ms and is settable in the CLI (new PG `servoDriverConfig` with CLI settings `sd_link_timeout_ms` (default 500) and `sd_boot_grace_ms` (default 2000); approved D6).
- **Required master frame rate:** >= 50 Hz. A false failsafe then needs 25 frames lost in a row. Below 20 Hz the board reports it (STATUSTEXT warning, at most every 10 s).
- **Recovery:** with no valid frame for 1 s, the parser state is reset every 1 s. With none for 2 s, the UART is re-initialised every 2 s (clears overrun/framing error states, which a hardware fault could leave behind). Approved (D7).
- **Master side (later):** on missing board HEARTBEAT for > 3 s, reopen the port; react to `FAILSAFE` STATUSTEXT.

## 9. Test plan
**SITL (`test/test_05_arming_safety.py`):**
- every transition in section 3, with its timing;
- arm-edge rule: checks apply on an observed disarmed -> armed edge; the first frame of a session saying ARMED arms at once (incl. right after boot, inside the boot grace);
- each own-health check (P1-P5): reject + STATUSTEXT reason, stays disarmed while ARMED keeps coming, arms on the next edge once the cause is gone;
- config commands refused while armed and in failsafe;
- FAILSAFE never times out (stays FAILSAFE for 60 s, then recovers with an armed frame);
- continuous motor writes at 200 Hz in every state (needs a SITL hook that counts motor writes);
- HEARTBEAT `system_status` per state; SYS_STATUS counters;
- parser reset after 1 s of silence.

**Bench (hardware only):**
- IWDG reset / brownout while armed -> reboot -> first frame ARMED re-arms at once;
- `save` while disarmed does not trigger the watchdog;
- ESC behaviour on failsafe and recovery;
- UART noise, cable pull and break condition;
- scope check of the continuous motor pulse.

## 10. Implementation notes (what the sections above do not state)
- STATUSTEXT strings: `ARM REJECTED: LINK_NOT_READY` / `BOOT_GRACE` / `MOTOR_CONFIG` / `CONFIG_ACTIVE`; `FAILSAFE: link lost`; `FAILSAFE: link restored`; `LINK: low frame rate`.
- Reject reason precedence when several checks fail: P1 > P2 > P3 > P4/P5. One text per rejected edge, at most 2 Hz.
- `FAILSAFE: link restored` is sent on any exit from FAILSAFE (armed or disarmed frame).
- `LINK: low frame rate` (WARNING): fewer than 20 AUX frames in a full 1 s window, at most every 10 s.
- `ARMING_FLAG(ARMED)` stays set in FAILSAFE; the config lock (section 5) uses it.
- SYS_STATUS counters are uint16 and wrap. `drop_rate_comm` counts sequence gaps over all messages from the master.
- Config PG: `servoDriverConfig`, `PG_SERVO_DRIVER_CONFIG` 557 (`pg_ids.h`), registered in `servo_driver.c`. `sd_link_timeout_ms` 100-5000, `sd_boot_grace_ms` 0-10000.
- IWDG: prescaler 32, reload 312 (250 ms nominal), STM32F10X only; fed every task cycle and in `config_streamer` erase/program. It also keeps running after CLI `bl` (TODO J).
- UART re-init after 2 s silence is skipped on SITL.
- SITL motor write rate is about 150/s (SITL task rate); 200 Hz to be verified on the bench.
