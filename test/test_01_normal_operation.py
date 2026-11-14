#!/usr/bin/env python3
"""Servo driver on SITL: normal operation with a healthy MAVLink link.

  test/test_01_normal_operation.py [--build] [--keep] [--verbose]

Scenarios:
 1. Start-up: servos 1500, motors 1000, HEARTBEAT about 1 Hz, disarmed
 2. Disarmed frames: servos follow, motors stay at min_command
 3. Arm / disarm: motors follow the armed frame, zero at once on the disarm frame, HEARTBEAT armed flag
 4. Continuous 50 Hz stream with a servo sweep: monotonic, many distinct values, motors without glitch
 5. Frame rates 10, 50, 100 Hz armed: outputs follow, no disarm
 6. Frame -> output latency
 7. Clamps and special values, servos 7-8 ignored
 8. arming_status 2 / 255 counts as disarmed
 9. Arm/disarm toggling at every frame (50 Hz, 2 s)

Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from common import harness as h
from common.harness import RUN, POSE, IDLE, check


def s1_startup(fx):
    """Start-up state and HEARTBEAT rate"""
    check("servos at servo_center_pulse", fx.servos() == [h.CENTRE] * h.SERVOS, str(fx.servos()))
    check("motors at min_command", fx.motors() == IDLE, str(fx.motors()))
    t0 = time.monotonic()
    n0 = fx.link.heartbeats
    time.sleep(3.0)
    count = len(fx.link.heartbeats_between(t0, time.monotonic()))
    check("HEARTBEAT about 1 Hz (2-4 in 3 s)", 2 <= count <= 4, "%d in 3 s" % count)
    check("HEARTBEAT reports disarmed", fx.link.board_armed is False, str(fx.link.board_armed))
    del n0


def s2_disarmed(fx):
    """Disarmed frames: servos follow, motors stay at min_command"""
    t0 = time.monotonic()
    fx.link.frames(1.0, False, [1400, 1400, 1400], POSE)
    check("servos follow servo 1-6 although disarmed", fx.servos() == POSE, str(fx.servos()))
    check("motors stay at min_command", fx.motors() == IDLE, str(fx.motors()))
    seen = set().union(*(fx.sitl.seen("motor", i, t0) for i in range(h.MOTORS)))
    check("motors never left min_command", seen == {h.MOTOR_MIN}, str(sorted(seen)))


def s3_arm_disarm(fx):
    """Arm and disarm"""
    pose2 = [1100, 1900, 1500, 1400, 1300, 1650]
    fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN)
    check("motors follow the armed frame", wait is not None, "after %s ms" % (None if wait is None else round(wait * 1e3)))
    fx.link.frames(1.2, True, RUN, POSE)
    check("HEARTBEAT reports armed", fx.link.stream_until_heartbeat(True, RUN, POSE) is True)
    fx.link.send_aux(False, RUN, pose2)
    wait = fx.wait_outputs(motors=IDLE, servos=pose2)
    check("disarm frame: motors min_command at once, servos follow", wait is not None,
          "after %s ms, motors %s servos %s" % (None if wait is None else round(wait * 1e3), fx.motors(), fx.servos()))
    fx.link.frames(0.5, False, RUN, pose2)
    check("servos keep following while disarmed", fx.servos() == pose2 and fx.motors() == IDLE)
    check("HEARTBEAT reports disarmed again", fx.link.stream_until_heartbeat(False, RUN, pose2) is False)


def s4_sweep(fx):
    """Servo sweep at 50 Hz, motors constant"""
    fx.settle(True)
    since = time.monotonic()
    fx.link.frames(3.0, True, RUN, POSE, step=lambda f: (True, RUN, [int(1000 + 1000 * f)] + POSE[1:]))
    sweep = fx.sitl.changes("servo", 0, since)
    check("servo 1 changes continuously", len(set(sweep)) >= 100, "%d distinct values" % len(set(sweep)))
    check("servo 1 sweep is monotonic", sweep == sorted(sweep), "first %s last %s" % (sweep[:1], sweep[-1:]))
    check("servo 1 reaches the end of the range", bool(sweep) and sweep[-1] >= 1950, "last %s" % sweep[-1:])
    others = [fx.sitl.changes("servo", i, since) for i in range(1, h.SERVOS)]
    check("servos 2-6 did not move", all(not c for c in others), str(others))
    glitch = {i: sorted(fx.sitl.seen("motor", i, since)) for i in range(h.MOTORS)}
    check("motors never changed (no glitch)", all(v == [RUN[i]] for i, v in glitch.items()), str(glitch))


def s5_rates(fx):
    """Frame rates 10, 50, 100 Hz while armed"""
    for rate, motors, pose in ((10, [1300, 1400, 1450], [1111, 1222, 1333, 1444, 1555, 1666]),
                               (50, [1350, 1450, 1550], [1212, 1312, 1412, 1512, 1612, 1712]),
                               (100, [1250, 1350, 1650], [1010, 1110, 1210, 1310, 1410, 1510])):
        fx.settle(True)
        t0 = time.monotonic()
        fx.link.frames(1.5, True, motors, pose, rate_hz=rate)
        ok = fx.servos() == pose and fx.motors() == motors
        check("%d Hz: outputs follow" % rate, ok, "servos %s motors %s" % (fx.servos(), fx.motors()))
        seen = set().union(*(fx.sitl.seen("motor", i, t0) for i in range(h.MOTORS)))
        allowed = set(RUN) | set(motors)
        check("%d Hz: motors never dropped to min_command" % rate, h.MOTOR_MIN not in seen, str(sorted(seen & {h.MOTOR_MIN})))
        check("%d Hz: board still armed" % rate, fx.link.stream_until_heartbeat(True, motors, pose) is True)
        del allowed


def s6_latency(fx):
    """Frame -> output latency"""
    fx.settle(True)
    sent = []
    for k in range(50):
        value = 1100 + 10 * k
        sent.append((fx.link.send_aux(True, RUN, [value] + POSE[1:]), value))
        time.sleep(0.03 + 0.001 * (k % 7))
    time.sleep(0.1)
    latencies = []
    for t_sent, value in sent:
        hits = [t for t, v in fx.sitl.events("servo", 0, t_sent) if v == value]
        if hits:
            latencies.append(hits[0] - t_sent)
    check("all 50 frames visible at the output", len(latencies) == 50, "%d of 50" % len(latencies))
    if latencies:
        med, mx = h.median(latencies) * 1e3, max(latencies) * 1e3
        check("latency median/max < 30 ms", mx < 30.0, "median %.1f ms, max %.1f ms, min %.1f ms" % (med, mx, min(latencies) * 1e3))


def s7_clamps(fx):
    """Clamps, no-value markers, servos 7-8"""
    fx.settle(True)
    link = fx.link
    link.frames(0.4, True, [2500, 500, 0], [2600, 400, 0, None, 1500, 1550, 1222, 1333])
    check("servo 2600/400 -> 2500/500", fx.servos()[:2] == [2500, 500], str(fx.servos()))
    check("servo value 0 and 65535 keep the previous value", fx.servos()[2:4] == POSE[2:4], str(fx.servos()))
    check("motor above max_throttle -> 2000, below min_command -> 1000, 0 -> 1000", fx.motors() == [2000, 1000, 1000], str(fx.motors()))
    link.frames(0.4, True, [None, 3000, 1400], [1900, 1000, None, 0, 1500, 1350, 2222, 2222])
    check("servos follow valid values, 0/65535 keep", fx.servos() == [1900, 1000, POSE[2], POSE[3], 1500, 1350], str(fx.servos()))
    check("motor 65535 -> 1000, 3000 -> 2000, 1400 applied", fx.motors() == [1000, 2000, 1400], str(fx.motors()))
    since = time.monotonic()
    link.frames(0.4, True, RUN, [1700, 1600, 1500, 1400, 1300, 1250, 500, 500])
    link.frames(0.4, True, RUN, [1700, 1600, 1500, 1400, 1300, 1250, 2500, 2500])
    check("servos 7-8 have no effect on outputs 1-6", fx.servos() == [1700, 1600, 1500, 1400, 1300, 1250]
          and len(fx.sitl.changes("servo", 0, since + 0.05)) == 0, str(fx.servos()))
    check("no outputs beyond servo 6 / motor 3", set(fx.sitl.values("servo")) <= set(range(h.SERVOS))
          and set(fx.sitl.values("motor")) <= set(range(h.MOTORS)))


def s8_arming_values(fx):
    """arming_status 2 and 255 count as disarmed"""
    for raw in (2, 255):
        fx.settle(True)
        check("arming_status=1 runs the motors", fx.motors() == RUN, str(fx.motors()))
        fx.link.send_aux(True, RUN, POSE, arming_raw=raw)
        wait = fx.wait_outputs(motors=IDLE)
        check("arming_status=%d: motors min_command" % raw, wait is not None, str(fx.motors()))
        fx.link.frames(0.4, True, RUN, POSE, arming_raw=raw)
        check("arming_status=%d: still min_command while the stream continues" % raw, fx.motors() == IDLE, str(fx.motors()))
        check("arming_status=%d: HEARTBEAT disarmed" % raw,
          fx.link.stream_until_heartbeat(True, RUN, POSE, arming_raw=raw) is False)


def s9_toggle(fx):
    """Arm/disarm toggling at every frame, 50 Hz, 2 s"""
    fx.settle(False)
    since = time.monotonic()
    state = [False]

    def toggle(_fraction):
        state[0] = not state[0]
        return state[0], RUN, POSE

    fx.link.frames(2.0, False, RUN, POSE, step=toggle)
    seen = set().union(*(fx.sitl.seen("motor", i, since) for i in range(h.MOTORS)))
    check("motors only took min_command or the commanded values", seen <= set(RUN) | {h.MOTOR_MIN}, str(sorted(seen)))
    fx.link.send_aux(False, RUN, POSE)
    check("final disarm frame: no stuck motor", fx.wait_outputs(motors=IDLE) is not None, str(fx.motors()))
    fx.link.send_aux(True, RUN, POSE)
    check("final armed frame: motors follow", fx.wait_outputs(motors=RUN) is not None, str(fx.motors()))
    fx.link.send_aux(False, RUN, POSE)
    check("disarm again", fx.wait_outputs(motors=IDLE) is not None, str(fx.motors()))


SCENARIOS = [s1_startup, s2_disarmed, s3_arm_disarm, s4_sweep, s5_rates, s6_latency, s7_clamps, s8_arming_values,
             s9_toggle]

if __name__ == "__main__":
    sys.exit(h.run_scenarios(h.parse_args(__doc__), SCENARIOS))
