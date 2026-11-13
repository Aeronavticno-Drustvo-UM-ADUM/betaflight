# SATELLITEBOARD arming and safety specification

Status: **APPROVED and IMPLEMENTED 2026-11-11; arming rule revised 2026-11-13 (owner: "the master is primary").** Owner decisions D1-D8 below.
Scope: `src/main/io/servo_driver.c` and the paths it controls. Open risks: `../TODO.md`. Tests: `../test/README.md`.
Master (flight controller) requirements: section 11.

## The rule in five lines (owner, 2026-11-13, D8)
1. **The flight controller is the master. Master ARMED = satellite ARMED, no questions asked.** The satellite never refuses an ARMED frame.
2. **The only thing that stops the motors without a master command is total link loss** (no valid frame for `T_fs`, 500 ms): FAILSAFE. When frames return, the satellite follows the master again at once.
3. **The satellite reports whether it is ready to arm** (HEARTBEAT `system_status`: `STANDBY` ready, `UNINIT` not ready, reason in a STATUSTEXT).
4. **The master blocks its own arming while the satellite is not ready or not heard.** This is a pre-arm check only: once armed, the master ignores readiness.
5. **In flight nothing disarms the satellite:** satellite reset, short link loss, CRC errors and configurator activity never keep it disarmed while the master is armed.

## 1. Principles (owner, 2026-11-11)
1. **The master is the primary arming authority.** The satellite does not repeat the master's pre-arm checks: throttle position, sensors and so on are the master's job. The satellite checks only its own health: init done, link health, output configuration, no configurator session (section 4). It **reports** the result continuously; it never acts on it (D8). The master uses it as a pre-arm check (section 11).
2. **Armed:** the board follows the master. The only event that disarms the motors without a master command is **loss of the link to the master**. Everything else is reported, never acted on by disarming. A wrong disarm in flight means losing the aircraft.
3. **Link loss detection must not give false alarms.** Recovering the link is part of the design.
4. Servos always follow valid commands, in every state. In failsafe they hold their last position.
5. Every rule here has a test in `test/test_05_arming_safety.py` (SITL) or in a bench test (hardware-only items).

## 2. States
| State | Motors | Servos | HEARTBEAT `base_mode` / `system_status` |
|---|---|---|---|
| `BOOT` (init up to the first task cycle) | `min_command` | `servo_center_pulse` | (none sent) |
| `DISARMED`, ready (section 4 all pass) | `min_command` | follow valid frames | 0 / `MAV_STATE_STANDBY` |
| `DISARMED`, not ready | `min_command` | follow valid frames | 0 / `MAV_STATE_UNINIT` |
| `ARMED` | follow armed frames, clamped | follow valid frames | `SAFETY_ARMED` / `MAV_STATE_ACTIVE` |
| `FAILSAFE` (armed intent, link lost) | `min_command` | hold the last valid value | `SAFETY_ARMED` cleared / `MAV_STATE_CRITICAL` |

The motor output is written **on every task cycle (200 Hz)** in every state, not only when a frame arrives. This fixes TODO A: the ESCs always see a continuous pulse train, including `min_command` in failsafe.

## 3. Transitions
| From | To | Condition | Notes |
|---|---|---|---|
| `BOOT` | `DISARMED` | init done, normal start (power-on / pin reset) | |
| `DISARMED` | `ARMED` | any valid frame says ARMED | immediately, **no checks, ready or not** (D8). Motors follow that same frame. Covers the arm edge, the first frame of a link session (board rebooted or link recovered while the master was armed) and the race where readiness drops between the master's check and its ARMED frame |
| `ARMED` | `DISARMED` | valid frame with `arming_status` != ARMED | immediate, by design |
| `ARMED` | `FAILSAFE` | no valid `ADUM_AUX_CHANNELS` for `T_fs` (default 500 ms) | HEARTBEAT and other messages do not count |
| `FAILSAFE` | `ARMED` | valid armed frame (first frame of the new link session) | in-flight recovery: **no checks**, the master decides (resolves TODO B) |
| `FAILSAFE` | `DISARMED` | valid disarmed frame | |

**Link session:** starts with the first valid `ADUM_AUX_CHANNELS` after boot or after a gap > `T_fs`. **D1 (owner): FAILSAFE never times out into DISARMED**; it ends only with a frame from the master.

## 4. Readiness (own-health checks, reported only)
Evaluated continuously. The result goes into the HEARTBEAT while `DISARMED` (section 2) and, while not ready, into a STATUSTEXT (section 7). **It never blocks arming on the satellite** (D8, was "reject on an observed arm edge" until 2026-11-13). In `ARMED` and `FAILSAFE` the HEARTBEAT shows the state, not the readiness.

| # | Check | Reason code |
|---|---|---|
| P1 | Link healthy: link session active, at least 5 consecutive valid AUX frames, no CRC error in the last 1 s. A frame with a message id unknown to the dialect is ignored, not a CRC error. | `LINK` |
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
- **STATUSTEXT**, severity WARNING, text `NOT READY: <reason code>` (section 4). Sent with every HEARTBEAT (1 Hz) while `DISARMED` and not ready; only the first failing check is named.
- **STATUSTEXT**, severity CRITICAL, text `FAILSAFE: link lost` / `FAILSAFE: link restored`. Sent on each transition.
- **SYS_STATUS** (1 Hz):
  - `errors_comm` = CRC/parse errors since boot;
  - `drop_rate_comm` = lost AUX frames over the last 1 s, from MAVLink sequence gaps, in 0.01 %;
  - `errors_count1` = TX drops; `errors_count2` = clamped values.
- No dialect change is needed for the arming rule (D8). A dedicated `ADUM_STATUS` message is not planned.

## 8. Link loss detection and recovery
- **Detection:** only a valid `ADUM_AUX_CHANNELS` (CRC OK, any sysid) refreshes the timer. `T_fs` defaults to 500 ms and is settable in the CLI (new PG `servoDriverConfig` with CLI settings `sd_link_timeout_ms` (default 500) and `sd_boot_grace_ms` (default 2000); approved D6).
- **Required master frame rate:** >= 50 Hz. A false failsafe then needs 25 frames lost in a row. Below 20 Hz the board reports it (STATUSTEXT warning, at most every 10 s).
- **Recovery:** with no valid frame for 1 s, the parser state is reset every 1 s. With none for 2 s, the UART is re-initialised every 2 s (clears overrun/framing error states, which a hardware fault could leave behind). Approved (D7).
- **Master side (later):** on missing board HEARTBEAT for > 3 s, reopen the port; react to `FAILSAFE` STATUSTEXT.

## 9. Test plan
**SITL (`test/test_05_arming_safety.py`):**
- every transition in section 3, with its timing;
- every ARMED frame arms at once (< 50 ms), ready or not: arm edge, first frame of a session (incl. right after boot, inside the boot grace), FAILSAFE recovery;
- each readiness check (P1-P5): HEARTBEAT `UNINIT` + `NOT READY: <code>` while the cause lasts, `STANDBY` and no text once it is gone; an ARMED frame during the cause still arms;
- a frame with an unknown message id does not make the board not ready;
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
- STATUSTEXT strings: `NOT READY: LINK` / `BOOT_GRACE` / `MOTOR_CONFIG` / `CONFIG_ACTIVE`; `FAILSAFE: link lost`; `FAILSAFE: link restored`; `LINK: low frame rate`.
- Readiness precedence when several checks fail: P1 > P2 > P3 > P4/P5.
- Unknown message id: the MAVLink parser fails the CRC of such a frame (no `crc_extra`). `servo_driver.c` checks `mavlink_get_msg_entry()` for the id of the failed frame and does not count it. A corrupted id that becomes unknown is not counted either (accepted).
- Diagnostics (`servoDriverDiag`, SWD): `armEdges` counts arm edges within a session, `edgesNotReady` those that came while not ready (armed anyway), `lastNotReadyReason` its reason; `notReadyTextsSent` / `notReadyTextsDropped`.
- `FAILSAFE: link restored` is sent on any exit from FAILSAFE (armed or disarmed frame).
- `LINK: low frame rate` (WARNING): fewer than 20 AUX frames in a full 1 s window, at most every 10 s.
- `ARMING_FLAG(ARMED)` stays set in FAILSAFE; the config lock (section 5) uses it.
- SYS_STATUS counters are uint16 and wrap. `drop_rate_comm` counts sequence gaps over all messages from the master.
- Config PG: `servoDriverConfig`, `PG_SERVO_DRIVER_CONFIG` 557 (`pg_ids.h`), registered in `servo_driver.c`. `sd_link_timeout_ms` 100-5000, `sd_boot_grace_ms` 0-10000.
- IWDG: prescaler 32, reload 312 (250 ms nominal), STM32F10X only; fed every task cycle and in `config_streamer` erase/program. It also keeps running after CLI `bl` (TODO J).
- UART re-init after 2 s silence is skipped on SITL.
- SITL motor write rate is about 150/s (SITL task rate); 200 Hz to be verified on the bench.

## 11. Master requirements (flight controller side, e.g. INAV)
What the master must do so that the rule above holds end to end. No dialect change: HEARTBEAT and STATUSTEXT from the common set.

1. **Read the satellite HEARTBEAT** (sysid/compid of the satellite) and keep `system_status` and the time it arrived. Satellite HEARTBEAT rate: 1 Hz.
2. **Pre-arm check "satellite not ready"** (an arming-disabled flag of the master). Arming is **blocked** unless:
   - a satellite HEARTBEAT arrived within the last 3 s (suggested; covers 2 lost HEARTBEATs), and
   - its `system_status` is `MAV_STATE_STANDBY` (3).
   Any other value blocks: `UNINIT` (0, not ready), `CRITICAL` (5, FAILSAFE), `ACTIVE` (4, still armed from before, e.g. after a master reboot).
3. **The check applies only while disarmed.** Like every other arming-disabled flag it may only prevent arming; it must never disarm, never trigger the master's failsafe and never limit outputs once armed.
4. **Show the reason** to the pilot: the last `NOT READY: <code>` STATUSTEXT (severity WARNING) from the satellite, for example in the arming-disabled reason / OSD / configurator. Codes: `LINK`, `BOOT_GRACE`, `MOTOR_CONFIG`, `CONFIG_ACTIVE`.
5. **Do not wait for a satellite reply after arming.** Send ARMED in `ADUM_AUX_CHANNELS` as today. The satellite follows within one frame. `ACTIVE` in the next HEARTBEAT confirms it (optional to display).
6. **Optional, information only:** `FAILSAFE: link lost` / `link restored` (CRITICAL) and HEARTBEAT `CRITICAL` while armed: warn the pilot, take no action.

Expected timeline on the ground: satellite power-on -> `UNINIT` (`LINK`, then `BOOT_GRACE` for 2 s) -> `STANDBY` -> master may arm. Opening the satellite CLI or saving its config -> `UNINIT` (`CONFIG_ACTIVE`) -> master cannot arm.

In flight: satellite reset or link loss -> satellite HEARTBEAT `UNINIT` or missing -> master stays armed and keeps sending -> satellite re-arms on the first frame.

## 12. Message ids and dialect compatibility (owner, 2026-11-13)
Goal: no message-id or dialect fault can make an armed satellite with a good link reject frames, disarm, or fail to re-arm in flight.

1. **Link liveness = valid `ADUM_AUX_CHANNELS` only.** No other message, known or unknown, refreshes or breaks the link timer.
2. **Everything else is ignored.** Known messages other than AUX, unknown ids, HEARTBEAT: no state change, not a link error. An unknown id only fails the CRC because the parser has no `crc_extra` for it; it is not counted (section 10).
3. **No filters that can reject the master.** AUX is accepted from any sysid/compid, MAVLink 2 signature not checked (TODO I stays as is), sequence gaps only counted. No such filter is to be added.
4. **Dialect evolution: never change an existing message.** Fields of `ADUM_AUX_CHANNELS` (id 13002) are frozen: a change alters `crc_extra`, so every AUX frame of the other side fails the CRC. New fields only as MAVLink 2 `<extensions>` (do not change `crc_extra`); new functions only as new message ids (the old side ignores them, rule 2).
5. **One dialect source.** `adum.xml` is identical in both repos (`lib/main/MAVLink_adum/adum.xml` here, `lib/main/MAVLink/adum.xml` in INAV). Both sides regenerate from it; `test/tools/check_mavlink_gen.sh` checks the frames byte for byte.
6. **A mismatch is caught on the ground, never in flight.** Dialects only change with a firmware update. With a mismatch no AUX frame is valid, so there is no link session: the satellite reports `NOT READY: LINK` (HEARTBEAT is a common message and still decodes) and the master cannot arm (section 11). A satellite that was never armed on this link cannot be armed by it.
7. **Port and bandwidth:** the satellite UART carries only master -> satellite AUX (+ optional HEARTBEAT) and satellite telemetry. The master must not route other telemetry streams to this port: extra traffic could delay AUX frames past `T_fs` (rate/baud check: TODO S9).

## 13. Uncommanded spin-up on the ground (owner, 2026-11-13: accepted, no logic change)
- **Satellite powered or connected while the master is already armed** (first frame ARMED arms at once, section 3): cannot occur in the aircraft. Master and satellite are fixed-connected, one battery powers both, and the satellite also gets 5 V over the master's USB, so both always boot together and the master boots disarmed.
- **ESCs powered after arming on USB** (battery plugged in while the master is armed on USB): covered by the master; it does not arm in that state.
- **Setup check:** `min_command` must be below the ESC start threshold. The satellite outputs it in DISARMED and FAILSAFE, so a wrong value spins the motors while disarmed.
