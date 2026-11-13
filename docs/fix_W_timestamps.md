# Fix W: int32 timestamp windows in `servo_driver.c` (2026-11-13)

Owner approved the fix, risk-based. Scope: `src/main/io/servo_driver.c` only. No change of thresholds, timing or state logic.

## Problem
`cmpTimeUs(a, b)` is `(int32_t)(a - b)`: a timestamp older than 2^31 us (35.8 min) reads as negative ("in the future"); the sign flips every 35.8 min.
Shown on the board (INAV `docs/adum/TODO.md` W, script `w0_stale_timestamp.py`): CRC error, uptime +50 min, arm edge ->
rejected `LINK_NOT_READY` **and** the reject text suppressed (stale `lastRejectTextUs`). Same signature as bench finding H1.

## Affected sites and fix

| # | Site (line, 2026-11-13) | Hazard when stale > 35.8 min | Fix |
|---|---|---|---|
| 1 | P1 `haveCrcError && cmp(now, lastCrcErrorUs) < 1 s` (289) | false `LINK_NOT_READY` reject | window expiry (below) |
| 2 | P4 `haveConfigActivity && cmp(now, lastConfigActivityUs) < 2 s` (303) | false `CONFIG_ACTIVE` reject | window expiry |
| 3 | reject text `!haveRejectText \|\| cmp(now, lastRejectTextUs) >= 0.5 s` (338) | reject text suppressed: silent reject | window expiry |
| 4 | low-rate text `!haveLowRateText \|\| cmp(now, lastLowRateTextUs) >= 10 s` (524) | warning suppressed | window expiry |
| 5 | low-rate eligibility `cmp(sessionStartUs, lastTickUs) <= 0` (523) | warning suppressed in a session older than 35.8 min | bool `sessionAtLastTick` |
| 6 | recovery `silentUs = cmp(now, lastAuxUs)` (425) | > 35.8 min silence (or no master since boot): parser reset and UART re-init stop | clamp silence while no session |
| 7 | recovery `cmp(now, lastParserResetUs / lastUartResetUs) >= 1 s / 2 s` (426, 432) | link up > 35.8 min, then lost: first recovery delayed up to 35.8 min | refresh both on every valid AUX frame |
| 8 | 1 Hz tick `cmp(now, lastTickUs) >= 1 s` (521) | `lastTickUs` stale while `mavlinkPort == NULL` (no MAVLink port, failed re-open): HEARTBEAT/SYS_STATUS stop up to 35.8 min once the port exists | also fire when the difference is negative |

Not changed: link timeout (399, `lastAuxUs` is fresh while `sessionActive`, checked every cycle); boot grace (483, latched before any wrap).

### Window expiry (sites 1-4)
New `servoDriverExpireWindows(currentTimeUs)`, called **first** in `servoDriverUpdate()` (before the link timeout check). Per pair: `if (flag && cmpTimeUs(now, last) >= WINDOW) flag = false;`. The use sites then test the flag only.
Equivalence: every use runs later in the same `servoDriverUpdate()` call with the same `currentTimeUs`, so the decision is identical for differences < 2^31 us; the flag is cleared within one cycle after the window, so a stored timestamp is never compared when older than WINDOW + one cycle.

### Site 5
`static bool sessionAtLastTick`: set to `sessionActive` at the end of each 1 Hz tick; set to false when a new session starts (`servoDriverHandleAux`, `newSession`). Eligibility = `sessionActive && sessionAtLastTick` (= session started at or before the last tick). `sessionStartUs` is then unused: remove it.

### Sites 6-7
- In `servoDriverRecoverLink()`: when `!sessionActive` and `silentUs > SERVO_DRIVER_SILENCE_CAP_US` (10 s, new define), set `lastAuxUs = currentTimeUs - SERVO_DRIVER_SILENCE_CAP_US` and use the capped value. Runs every cycle, so `silentUs` never reaches 2^31. Only while no session: the link timeout (max 5000 ms, `sd_link_timeout_ms` range) is never affected; cap must stay > max link timeout and > UART re-init time (comment).
- In `servoDriverHandleAux()`, where `lastAuxUs` is set: also `lastParserResetUs = lastUartResetUs = currentTimeUs`. Equivalence: resets only happen during silence, so before the fix the "since last reset" condition was always true at the first reset; now it equals `silentUs`, same result.

### Site 8
`const int32_t sinceTickUs = cmpTimeUs(currentTimeUs, lastTickUs); if (sinceTickUs >= ONE_SECOND || sinceTickUs < 0)`: `lastTickUs` is never in the future, negative means overflow.

## Hazard checks (review list)
1. No timing change below 35.8 min: full Betaflight SITL suite (286 checks, incl. failsafe 505-516 ms, reject texts, low-rate text) passes unchanged.
2. Expiry runs before every use in the same cycle (call order in `servoDriverUpdate`).
3. Clamp of `lastAuxUs` only when `!sessionActive`; never delays failsafe; no re-arm path depends on `lastAuxUs` age.
4. `servoDriverNoteConfigActivity()` (cli.c, msp.c) unchanged; a timestamp set slightly after `currentTimeUs` gives a small negative difference = window active (as before).
5. Task not run for > 35.8 min: impossible on HW (IWDG 250 ms); on SITL no stall that long.
6. No new flash/RAM worth noting (one bool, one define).

## Verification
- SITL regression test with a SITL-only time-jump hook (one test per site where observable: 1, 3, 6/7 as far as SITL allows, 8 not needed); must fail before the fix, pass after.
- Full SITL suite, then on the board: W-0 script step C2 must arm (and a reject in step B must still be reported), `hw_output_check.py --armed` 3x.

## Separate, not part of W
- X (owner): a valid frame with an unknown message id is counted as a CRC error: arm rejected `LINK_NOT_READY` for 1 s (W-0 step B).
