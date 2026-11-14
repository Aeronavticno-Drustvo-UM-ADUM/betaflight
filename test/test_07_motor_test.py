#!/usr/bin/env python3
"""Servo driver on SITL: motor test (MOTOR_TEST, arming_status 2) of docs/arming_safety.md section 14 (MT1-MT10).

  test/test_07_motor_test.py [--build] [--keep] [--verbose]

Scenarios:
 1. MT1-MT3: MOTOR_TEST frames while disarmed: motors follow, clamped, no-value -> min_command, state stays DISARMED, text "MOTOR TEST"
 2. MT4: DISARMED frame stops the test at once, no frames stop it after 200 ms
 3. MT5: MOTOR_TEST in ARMED and FAILSAFE only disarms, the test values never reach the motors
 4. MT8/MT10: MOTOR_TEST ignored for 5 s after a disarm from ARMED, one "MOTOR TEST BLOCKED" text
 5. MT9/MT10: MOTOR_TEST ignored until the boot grace is over after a restart
 6. Arming during a motor test: ARMED frame arms, motors follow the ARMED frame only
Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common import mavlink_gen as mav  # noqa: E402
from common.harness import RUN, POSE, IDLE, check  # noqa: E402

MOTOR_TEST = mav.ADUM_ARMING_STATUS_MOTOR_TEST
BOOT_GRACE_S = 2.0                 # sd_boot_grace_ms default
TEST_TIMEOUT_S = 0.200             # SERVO_DRIVER_MOTOR_TEST_TIMEOUT_MS
LOCKOUT_S = 5.0                    # SERVO_DRIVER_MOTOR_TEST_LOCKOUT_MS
MOTOR_LATENCY_S = 0.05
TIMEOUT_MARGIN_S = 0.100           # margin on top of the 200 ms (task cycle, desktop load)
TEST_VALUES = [1200, 1400, 1600]   # differ from RUN in every motor
SEV_WARNING = mav.MAV_SEVERITY_WARNING
STANDBY, ACTIVE = mav.MAV_STATE_STANDBY, mav.MAV_STATE_ACTIVE

measured = {"timeout": [], "lockout_end": [], "grace_end": []}


# ---------------------------------------------------------------- helpers
def test_frames(fx, seconds, motors=None):
    """MOTOR_TEST frames at 50 Hz."""
    motors = TEST_VALUES if motors is None else motors
    fx.link.frames(seconds, False, motors, POSE, arming_raw=MOTOR_TEST)


def plain_frames(fx, seconds, armed, motors=RUN):
    """ARMED or DISARMED frames at 50 Hz."""
    fx.link.frames(seconds, armed, motors, POSE)


def wait_boot(fx):
    """Disarmed frames until the boot grace is over."""
    while fx.since_boot() < BOOT_GRACE_S + 0.4:
        plain_frames(fx, 0.1, False)


def motor_changes_since(fx, since):
    """Every motor value written after `since` (all motors)."""
    values = set()
    for index in range(h.MOTORS):
        values.update(fx.sitl.changes("motor", index, since))
    return values


def texts(fx, since, text):
    """STATUSTEXTs with exactly this text since `since`."""
    return [x for x in fx.link.texts_since(since, text) if x[2] == text]


def state_status(fx, armed, keep_test=False):
    """HEARTBEAT system_status of the next HEARTBEAT, frames keep coming."""
    if keep_test:
        keep = lambda: test_frames(fx, 0.1)
    else:
        keep = lambda: plain_frames(fx, 0.1, armed)
    return fx.link.status_after(time.monotonic(), 2.5, keep)


def go_disarmed_idle(fx):
    """Known state: DISARMED, link healthy, motors at min_command."""
    plain_frames(fx, 0.4, False)


# ---------------------------------------------------------------- scenarios
def s1_motor_test_follows(fx):
    """MT1-MT3: MOTOR_TEST frames while disarmed: motors follow, clamps, state stays DISARMED"""
    wait_boot(fx)
    go_disarmed_idle(fx)
    check("start: motors min_command", fx.motors() == IDLE, str(fx.motors()))

    t0 = time.monotonic()
    for step, values in enumerate(([1200, 1400, 1600], [1300, 1500, 1700])):
        fx.link.send_aux(False, values, POSE, MOTOR_TEST)
        wait = fx.wait_outputs(motors=values, timeout=0.5)
        check("MT3: test values %s reach the motors (< 50 ms)" % values,
              wait is not None and wait < MOTOR_LATENCY_S, "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
        test_frames(fx, 0.2, values)

    status = state_status(fx, False, keep_test=True)
    check("MT2: HEARTBEAT STANDBY and SAFETY_ARMED clear during the motor test",
          status == STANDBY and fx.link.board_armed is False, "status %s armed %s" % (status, fx.link.board_armed))
    start_texts = texts(fx, t0, "MOTOR TEST")
    check("MT6: STATUSTEXT 'MOTOR TEST' received", len(start_texts) >= 1, "%d texts" % len(start_texts))
    check("MT6: severity WARNING (4)", bool(start_texts) and start_texts[0][1] == SEV_WARNING,
          str([x[1] for x in start_texts[:2]]))
    check("MT10: no 'MOTOR TEST BLOCKED' text", not texts(fx, t0, "MOTOR TEST BLOCKED"))

    clamp_cases = [("above max_throttle -> max_throttle", [2500, 2200, 2100], [h.MOTOR_MAX] * 3),
                   ("below min_command -> min_command", [900, 800, 950], [h.MOTOR_MIN] * 3),
                   ("no value (65535, 0) -> min_command", [None, 0, 65535], [h.MOTOR_MIN] * 3)]
    for label, sent, expected in clamp_cases:
        test_frames(fx, 0.3, TEST_VALUES)
        test_frames(fx, 0.3, sent)
        check("MT3: %s" % label, fx.motors() == expected, "sent %s motors %s" % (sent, fx.motors()))
    status = state_status(fx, False, keep_test=True)
    check("MT2: still DISARMED after the clamp cases", status == STANDBY and fx.link.board_armed is False,
          "status %s armed %s" % (status, fx.link.board_armed))


def s2_motor_test_stops(fx):
    """MT4: DISARMED frame stops the test at once, silence stops it after 200 ms"""
    test_frames(fx, 0.4)
    check("test running: motors follow the test values", fx.motors() == TEST_VALUES, str(fx.motors()))
    fx.link.send_aux(False, RUN, POSE)                 # DISARMED frame with motor values that must be ignored
    wait = fx.wait_outputs(motors=IDLE, timeout=0.5)
    check("MT4: DISARMED frame -> motors min_command at once (< 50 ms)", wait is not None and wait < MOTOR_LATENCY_S,
          "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
    plain_frames(fx, 0.3, False)
    check("MT4: motors stay min_command with DISARMED frames", fx.motors() == IDLE, str(fx.motors()))

    test_frames(fx, 0.4)
    check("test running again", fx.motors() == TEST_VALUES, str(fx.motors()))
    t_last = fx.link.last_aux_t
    time.sleep(0.6)                                    # no frames at all
    times = fx.disarm_times(t_last)
    delays = [t - t_last for t in times if t is not None]
    measured["timeout"].extend(delays)
    check("MT4: no frames -> every motor min_command within 200 ms + margin",
          len(delays) == h.MOTORS and TEST_TIMEOUT_S - 0.01 <= min(delays) and max(delays) <= TEST_TIMEOUT_S + TIMEOUT_MARGIN_S,
          "%s ms" % [round(d * 1e3) for d in delays])
    plain_frames(fx, 0.3, False)
    check("MT4: link back with DISARMED frames: motors stay min_command", fx.motors() == IDLE, str(fx.motors()))


def s3_motor_test_in_flight(fx):
    """MT5: MOTOR_TEST while ARMED or in FAILSAFE only disarms, no test values on the motors"""
    plain_frames(fx, 0.4, True)
    check("ARMED: motors follow the armed frame", fx.motors() == RUN, str(fx.motors()))
    t_send = fx.link.send_aux(False, TEST_VALUES, POSE, MOTOR_TEST)
    wait = fx.wait_outputs(motors=IDLE, timeout=0.5)
    check("MT5: MOTOR_TEST frame while ARMED -> motors min_command at once (< 50 ms)",
          wait is not None and wait < MOTOR_LATENCY_S, "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
    test_frames(fx, 0.5)
    values = motor_changes_since(fx, t_send)
    check("MT5: test values never reach the motors (ARMED)", values <= {h.MOTOR_MIN}, str(sorted(values)))
    status = state_status(fx, False, keep_test=True)
    check("MT5: satellite DISARMED (HEARTBEAT STANDBY, not armed)", status == STANDBY and fx.link.board_armed is False,
          "status %s armed %s" % (status, fx.link.board_armed))

    # FAILSAFE: arm, then no frames for longer than the link timeout
    plain_frames(fx, 0.4, True)
    check("FAILSAFE setup: armed, motors follow", fx.motors() == RUN, str(fx.motors()))
    fx.link.gap(h.FAILSAFE_TIMEOUT_S + 0.5)
    check("FAILSAFE setup: motors min_command after the link loss", fx.motors() == IDLE, str(fx.motors()))
    t_recover = time.monotonic()
    test_frames(fx, 0.5)
    values = motor_changes_since(fx, t_recover)
    check("MT5: test values never reach the motors (FAILSAFE recovery)", values <= {h.MOTOR_MIN}, str(sorted(values)))
    check("MT5: motors min_command after the recovery", fx.motors() == IDLE, str(fx.motors()))
    status = state_status(fx, False, keep_test=True)
    check("MT5: recovered to DISARMED (HEARTBEAT STANDBY, not armed)", status == STANDBY and fx.link.board_armed is False,
          "status %s armed %s" % (status, fx.link.board_armed))


def s4_lockout_after_disarm(fx):
    """MT8/MT10: MOTOR_TEST ignored for 5 s after a disarm from ARMED, one BLOCKED text"""
    plain_frames(fx, 0.4, True)
    t_disarm = fx.link.send_aux(False, RUN, POSE)
    plain_frames(fx, 0.1, False)
    while time.monotonic() < t_disarm + 4.0:
        test_frames(fx, 0.1)
    values = motor_changes_since(fx, t_disarm)
    check("MT8: motors stay min_command for 4 s of MOTOR_TEST frames", values <= {h.MOTOR_MIN} and fx.motors() == IDLE,
          "values %s motors %s" % (sorted(values), fx.motors()))
    blocked = texts(fx, t_disarm, "MOTOR TEST BLOCKED")
    check("MT10: exactly one 'MOTOR TEST BLOCKED' text", len(blocked) == 1, "%d texts" % len(blocked))
    check("MT10: severity WARNING (4)", bool(blocked) and blocked[0][1] == SEV_WARNING, str([x[1] for x in blocked]))
    check("MT8: no 'MOTOR TEST' start text while blocked", not texts(fx, t_disarm, "MOTOR TEST"))

    while time.monotonic() < t_disarm + LOCKOUT_S + 0.4:
        test_frames(fx, 0.1)
    check("MT8: after > 5 s the test works", fx.motors() == TEST_VALUES, str(fx.motors()))
    first_event = [t for index in range(h.MOTORS) for t, v in fx.sitl.events("motor", index, t_disarm) if v in TEST_VALUES]
    if first_event:
        measured["lockout_end"].append(min(first_event) - t_disarm)
        check("MT8: motors moved not earlier than 5 s after the disarm", min(first_event) - t_disarm >= LOCKOUT_S - 0.05,
              "%.2f s" % (min(first_event) - t_disarm))
    check("MT6: 'MOTOR TEST' start text after the lockout", len(texts(fx, t_disarm + LOCKOUT_S - 0.05, "MOTOR TEST")) >= 1)
    check("MT10: still only one BLOCKED text (episode ended with the lockout)",
          len(texts(fx, t_disarm, "MOTOR TEST BLOCKED")) == 1)


def s5_boot_grace(fx):
    """MT9/MT10: MOTOR_TEST ignored until the boot grace is over after a restart"""
    restarted = fx.restart()
    check("SITL restarted, MAVLink port open", restarted)
    if not restarted:
        return
    t0 = time.monotonic()
    while fx.since_boot() < BOOT_GRACE_S - 0.4:
        test_frames(fx, 0.05)
    values = motor_changes_since(fx, t0)
    check("MT9: motors stay min_command inside the boot grace (%.1f s after boot)" % fx.since_boot(),
          values <= {h.MOTOR_MIN} and fx.motors() == IDLE, "values %s motors %s" % (sorted(values), fx.motors()))
    blocked = texts(fx, t0, "MOTOR TEST BLOCKED")
    check("MT10: one 'MOTOR TEST BLOCKED' text inside the boot grace", len(blocked) == 1, "%d texts" % len(blocked))
    while fx.since_boot() < BOOT_GRACE_S + 0.4:
        test_frames(fx, 0.05)
    check("MT9: after the boot grace the test works", fx.motors() == TEST_VALUES, str(fx.motors()))
    moved = [t for index in range(h.MOTORS) for t, v in fx.sitl.events("motor", index, t0) if v in TEST_VALUES]
    if moved:
        measured["grace_end"].append(min(moved) - fx.sitl.t_start)
        check("MT9: motors moved not before the end of the boot grace", min(moved) - fx.sitl.t_start >= BOOT_GRACE_S - 0.05,
              "%.2f s after boot" % (min(moved) - fx.sitl.t_start))
    check("MT10: blocked episode ended, still one BLOCKED text", len(texts(fx, t0, "MOTOR TEST BLOCKED")) == 1)
    check("MT6: 'MOTOR TEST' start text after the grace", len(texts(fx, t0, "MOTOR TEST")) >= 1)


def s6_arm_during_test(fx):
    """Arming during a motor test: ARMED frame arms, motors follow the ARMED frame only"""
    test_frames(fx, 0.4)
    check("test running", fx.motors() == TEST_VALUES, str(fx.motors()))
    armed_values = [1550, 1650, 1750]
    t_arm = fx.link.send_aux(True, armed_values, POSE)
    wait = fx.wait_outputs(motors=armed_values, timeout=0.5)
    check("ARMED frame during the test: motors follow the ARMED frame (< 50 ms)",
          wait is not None and wait < MOTOR_LATENCY_S, "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
    plain_frames(fx, 0.5, True, armed_values)
    values = motor_changes_since(fx, t_arm)
    check("no test value carried over into ARMED", values <= set(armed_values), str(sorted(values)))
    status = state_status(fx, True)
    check("state ARMED (HEARTBEAT ACTIVE, SAFETY_ARMED)", status == ACTIVE and fx.link.board_armed is True,
          "status %s armed %s" % (status, fx.link.board_armed))
    plain_frames(fx, 0.3, False)
    check("disarm at the end: motors min_command", fx.motors() == IDLE, str(fx.motors()))


SCENARIOS = [s1_motor_test_follows, s2_motor_test_stops, s3_motor_test_in_flight, s4_lockout_after_disarm,
             s5_boot_grace, s6_arm_during_test]

if __name__ == "__main__":
    code = h.run_scenarios(h.parse_args(__doc__), SCENARIOS)
    delays = measured["timeout"]
    if delays:
        print("motor test timeout over %d measurements: min %.0f ms, max %.0f ms" % (len(delays), min(delays) * 1e3, max(delays) * 1e3))
    for key, label in (("lockout_end", "first motor move after the disarm"), ("grace_end", "first motor move after boot")):
        if measured[key]:
            print("%s: %.2f s" % (label, measured[key][0]))
    sys.exit(code)
