# Open issues: SATELLITEBOARD servo driver

Open design decisions and risks found on 2026-11-11 while planning the SITL test suite. None of them is decided yet.
**Agents: before changing firmware behaviour in the areas below, or when a task touches them, ask the owner for the decision first. Do not pick a solution yourself.**

| # | Risk / open question | Effect | Proposed fix (not approved) | Status |
|---|---|---|---|---|
| A | Motor pulse written only when an `ADUM_AUX_CHANNELS` frame arrives. With ONESHOT125/MULTISHOT and `use_unsynced_pwm = OFF` (default, used by `test/servo_config.txt`), `pwmCompleteOneshotMotorUpdate` emits one pulse per `motorWriteAll` call. | ESC pulse rate = master frame rate (with its jitter). After failsafe, or disarmed without traffic, no pulses at all: ESC signal-loss mode, may beep or need re-arm, so recovery after link return can fail. | Write motors on every 200 Hz task cycle in `servo_driver.c`, or set `use_unsynced_pwm = ON`. Needs: ESC type and protocol. | decided 2026-11-11; implemented 2026-11-11, SITL-tested; bench pending |
| B | Re-arm right after failsafe: the first armed frame after link loss restarts motors at the commanded throttle, no handshake. | Throttle step when the link returns; on the ground, motors spin up immediately after a link glitch. | Keep (current), or require a disarmed frame after failsafe before re-arming. | decided 2026-11-11; implemented 2026-11-11, SITL-tested; bench pending |
| C | No watchdog (no IWDG code). | Firmware hang: hardware timers keep outputting; with unsynced PWM the motors keep their last throttle forever. | Enable the IWDG on the F103. | decided 2026-11-11; implemented 2026-11-11, not testable on SITL, bench pending |
| D | Reset in flight (brownout, watchdog). | Servos jump to `servo_center_pulse` (1500 us) until the first frame; boot ~1-2 s; ESC signal drop during boot. | Accept, or shorten the boot path. | decided 2026-11-11: accepted; after any reset the first ARMED frame re-arms at once (docs/arming_safety.md sec 3, 6) |
| E | Corrupt/reset EEPROM or `defaults`. | MAVLink serial function not configured: board ignores the master permanently (servos centre, motors zero). | MAVLink on UART2 as target default serial config. | open: ask owner. Phase 3 test |
| F | Arming bypasses the Betaflight arming-disable flags (`ENABLE_ARMING_FLAG` directly). | Board arms with the configurator connected, CLI open or motor tab in use. Unchecked whether CLI `save`/`reboot`/`defaults` are refused while armed. | Test first (phase 2), then decide. | decided 2026-11-11; implemented 2026-11-11, SITL-tested; bench pending |
| G | TX buffer full: HEARTBEAT / `ADUM_GPS_PVT` dropped silently. | Master sees gaps, no counter. | Drop counter, low priority. | open, low priority |
| H | A HEARTBEAT sent in the task cycle that detects the failsafe timeout still reports armed. `servoDriverUpdate` sends the heartbeat before the timeout check. Seen in SITL in about 1 of 2 runs. | For up to 1 s the master sees "armed" although the motors are already at zero. | Run the timeout check before the heartbeat send. Trivial change. | decided 2026-11-11; implemented 2026-11-11, SITL-tested; bench pending |
| I | Signed MAVLink frames (incompat flag 0x01) are accepted without a signature check. | No authentication of the master; irrelevant while no other nodes are on the bus. | None for now. | open, low priority. `[KNOWN]` in `test/test_03_frame_integrity.py` |
| J | IWDG keeps running after CLI `bl` (reboot to the ST system bootloader). | Bootloader resets after ~250 ms: serial bootloader flashing impossible; SWD unaffected. | Block `bl`, or do not start the IWDG / check a bootloader-request flag before the IWDG start. | open: ask owner |
| K | SITL startup race: dyad is not thread-safe; the main thread registers listeners while the tcp thread runs `dyad_update`. | About 1 in 3-4 SITL starts crash (SIGSEGV) or lose a listener; the test harness detects it and restarts. | Mutex in `src/main/drivers/serial_tcp.c` (SITL-only upstream code). | open: ask owner |
| L | P5 (config save pending) and UART re-init after 2 s silence are not testable on SITL. | No automated coverage. | Bench tests. | open: bench |
| M | Stale or forged ARMED frame (replay, buffering radio) arms the board: arm edge with checks passing, or first frame of a session without checks. INAV SITL test 09 (finding F2). | 30-86 ms motor pulse while the master is disarmed; ~500 ms if nothing follows. | Frame authentication / freshness (relates to I). | open: ask owner |
| N | After a stall, frames left in the UART RX buffer (128 B) are applied; failsafe timeout and re-arm on 1-5 s old ARMED frames in the same cycle. INAV SITL test 16 (F3). | Stale commands applied; IWDG covers stalls > ~250 ms on hardware, shorter stalls not. | Flush RX / discard frames older than the gap on recovery. | open: ask owner |
| O | Re-arm after FAILSAFE or reboot steps motors from min_command to the commanded throttle in one frame. INAV SITL tests 06/07/10/11 (F4). | ~500 us throttle step (by design, B). | Ramp on the board or master; keep. | open: ask owner |
| P | Delays < T_fs and buffered bursts are applied as current; no latency/age detection. INAV SITL test 08 (F5). | Up to ~500 ms old commands applied silently. | Timestamp/age field in the dialect. | open: ask owner |
| Q | `errors_count2` (clamped) counts every no-value motor with < 3 motors and every master min_command < 1000. INAV SITL test 04 (F9). | Counter useless as error indicator. | Do not count no-value; or document. | open, low priority |
| R | One replayed frame with an old sequence number counts a gap of up to 255 -> drop_rate_comm ~96 %. INAV SITL test 09 (F10). | Misleading link quality. | Ignore backward sequence jumps. | open, low priority |

Decided (2026-11-11, owner):
- Arming is controlled by `ADUM_AUX_CHANNELS` only; incoming HEARTBEAT does not keep the board armed.
- Failsafe (no `ADUM_AUX_CHANNELS` for 500 ms, or a disarmed frame): motors to zero, servos hold their last position.
- No other MAVLink nodes on the bus for now: sysid/compid not checked, placeholder 1/1.
- By design: a single disarmed frame cuts the motors at once; no slew-rate limit on the board (master's job).
- The master-side (INAV) implementation is out of scope for now.
- The master is the primary arming authority. The satellite checks only its own health (init, link, output config, no configurator session), and only on a disarmed->armed edge it observes; the first frame of a link session saying ARMED arms at once. Spec: `docs/arming_safety.md`.
- FAILSAFE never times out into DISARMED.


# User added notes:

Armin / Disarming:
  - we need to separate the general safety logic of arming, depending on:
  - when system is not yet armed: then we need to block arming for any safety item
  - when system is already armed and something happens which should block arming, in that case we must carefully consider how to proceed, we don't want to disarm running system for almost nothing that can be survived, because if we disarm this means total loss of the aircraft. I think the only such even is when we have no connection from master. You think if there is anything else, and point it out to me. The way I see it is that we always follow arming commands from master (when already armed) if we have link. we will also have to be careful what decides we have no valid link to master -> we don't want to wrongly detect a bad link, when actually link is fine. also, we must consider if link drop situation happens, what is the best course of action to get the link restored?
  - when not armed, if sattelite board detects arming should be blocked, and arming command arrives from master, we need to report back rejected arming, and the reason for it (we don't have that yet in mavlink, will follow in the future). this will also drive some changes on the master code i guess.
  - overall: arming / disarming, related safety measures, emergency measures ans so on-> this  must all be well documented and tested


