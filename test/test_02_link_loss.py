#!/usr/bin/env python3
"""Servo driver on SITL: loss of the MAVLink link (failsafe) and recovery.

  test/test_02_link_loss.py [--build] [--keep] [--verbose]

Scenarios:
 1. Gaps below the 500 ms timeout (100, 300, 450 ms) while armed: no glitch, still armed
 2. Gaps above the timeout (600 ms, 2 s): disarm time, motors min_command, servos untouched, HEARTBEAT disarmed
 3. Recovery with a DISARMED frame first, then ARMED: motors only follow the armed frame
 4. Recovery directly with an ARMED frame: motors follow at once (intended, spec sec. 3)
 5. Flapping: 20 cycles of 200 ms stream and 450/550 ms gap
 6. HEARTBEAT only while armed: disarm after the timeout
 7. TCP disconnect / reconnect of the MAVLink port while armed
 8. Long outage (10 s) then recovery, HEARTBEAT keeps running

Failsafe timing: the disarm must be seen 495 to 600 ms after the last ADUM_AUX_CHANNELS.
Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common.harness import RUN, POSE, IDLE, check  # noqa: E402

DISARM_MIN_S, DISARM_MAX_S = 0.495, 0.600     # accepted disarm delay after the last frame (timeout 500 ms + 5 ms task + margin)
disarm_delays_s = []                            # every measured failsafe delay, printed at the end


def motors_seen(fx, since, until=float("inf")):
    return set().union(*(fx.sitl.seen("motor", i, since, until) for i in range(h.MOTORS)))


def failsafe_delay(fx, t_last, until=float("inf")):
    """(min, max) delay in s from the last frame to the motors reaching min_command; None when a motor did not."""
    times = fx.disarm_times(t_last, until)
    if any(t is None for t in times):
        return None
    delays = [t - t_last for t in times]
    return min(delays), max(delays)


def check_failsafe_delay(fx, label, t_last, until=float("inf")):
    delay = failsafe_delay(fx, t_last, until)
    if delay is None:
        check("%s: motors disarmed" % label, False, "motors %s" % fx.motors())
        return
    disarm_delays_s.extend(delay)
    check("%s: disarm 495-600 ms after the last frame" % label, DISARM_MIN_S <= delay[0] and delay[1] <= DISARM_MAX_S,
          "%.0f to %.0f ms" % (delay[0] * 1e3, delay[1] * 1e3))


def s1_short_gaps(fx):
    """Gaps below the timeout while armed"""
    for gap_ms in (100, 300, 450):
        fx.settle(True)
        t0 = time.monotonic()
        fx.link.frames(0.3, True, RUN, POSE)
        fx.link.gap(gap_ms / 1e3)
        actual = time.monotonic() - fx.link.last_aux_t
        fx.link.frames(0.5, True, RUN, POSE)
        seen = motors_seen(fx, t0)
        check("%d ms gap: motors never left the commanded values" % gap_ms, seen == set(RUN),
              "values %s, gap %.0f ms" % (sorted(seen), actual * 1e3))
        check("%d ms gap: board stays armed" % gap_ms, fx.link.stream_until_heartbeat(True, RUN, POSE) is True)


def s2_long_gaps(fx):
    """Gaps above the timeout while armed"""
    for gap_s in (0.6, 2.0):
        fx.settle(True)
        t_last = fx.link.last_aux_t
        fx.link.gap(gap_s)
        t_end = time.monotonic()
        check_failsafe_delay(fx, "%.1f s gap" % gap_s, t_last, t_end)
        check("%.1f s gap: motors min_command" % gap_s, fx.motors() == IDLE, str(fx.motors()))
        changes = [fx.sitl.changes("servo", i, t_last, t_end) for i in range(h.SERVOS)]
        check("%.1f s gap: servos had zero changes" % gap_s, all(not c for c in changes), str(changes))
        check("%.1f s gap: HEARTBEAT reports disarmed" % gap_s, fx.link.wait_heartbeat(t_last + 0.8, 2.0) is False)
        fx.link.frames(0.3, False, RUN, POSE)


def s3_recover_disarmed_first(fx):
    """Recovery: disarmed frame first, then armed"""
    new_pose = [1100, 1900, 1500, 1400, 1300]
    fx.settle(True)
    time.sleep(1.0)
    check("failsafe before the recovery", fx.motors() == IDLE, str(fx.motors()))
    t0 = time.monotonic()
    fx.link.send_aux(False, RUN, new_pose)
    wait = fx.wait_outputs(servos=new_pose)
    check("disarmed frame: servos follow at once", wait is not None, "after %s ms" % (None if wait is None else round(wait * 1e3)))
    fx.link.frames(0.3, False, RUN, new_pose)
    check("disarmed frames: no motor change", all(not fx.sitl.changes("motor", i, t0) for i in range(h.MOTORS)),
          str(fx.motors()))
    t1 = time.monotonic()
    fx.link.send_aux(True, RUN, new_pose)
    wait = fx.wait_outputs(motors=RUN)
    check("armed frame: motors follow", wait is not None, "after %s ms" % (None if wait is None else round(wait * 1e3)))
    fx.link.frames(0.3, True, RUN, new_pose)
    seen = motors_seen(fx, t0)
    check("no intermediate motor values", seen == {h.MOTOR_MIN} | set(RUN), str(sorted(seen)))
    del t1


def s4_recover_armed(fx):
    """Recovery directly with an armed frame (intended: in-flight recovery, no checks, docs/arming_safety.md sec. 3)"""
    fx.settle(True)
    time.sleep(1.0)
    t0 = time.monotonic()
    fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN)
    check("armed frame after failsafe: motors follow at once (< 50 ms)", wait is not None and wait < 0.05,
          "motors %s after %s ms" % (fx.motors(), None if wait is None else round(wait * 1e3)))
    fx.link.frames(0.3, True, RUN, POSE)
    seen = motors_seen(fx, t0)
    check("motors only min_command and the commanded values", seen <= {h.MOTOR_MIN} | set(RUN), str(sorted(seen)))


def s5_flapping(fx):
    """Flapping link: 20 cycles of 200 ms stream and a 450 / 550 ms gap"""
    fx.settle(True)
    t0 = time.monotonic()
    long_gaps = 0
    for cycle in range(20):
        gap = 0.45 if cycle % 2 == 0 else 0.55
        long_gaps += gap > FAILSAFE_S
        fx.link.frames(0.2, True, RUN, POSE)
        fx.link.gap(gap)
    fx.link.frames(0.3, True, RUN, POSE)
    events = fx.sitl.events("motor", 0, t0)
    disarms, previous = 0, RUN[0]
    for _, value in events:
        disarms += value == h.MOTOR_MIN and previous != h.MOTOR_MIN
        previous = value
    check("disarm events = number of gaps > 500 ms", disarms == long_gaps, "%d disarms, %d long gaps" % (disarms, long_gaps))
    check("outputs consistent at the end", fx.motors() == RUN and fx.servos() == POSE, "%s %s" % (fx.motors(), fx.servos()))
    check("SITL alive", fx.sitl.alive())


FAILSAFE_S = h.FAILSAFE_TIMEOUT_S


def s6_heartbeat_only(fx):
    """HEARTBEAT alone does not keep the board armed"""
    fx.settle(True)
    t_last = fx.link.last_aux_t
    fx.link.heartbeats_only(1.2)
    check_failsafe_delay(fx, "heartbeats only", t_last)
    check("motors min_command", fx.motors() == IDLE, str(fx.motors()))


def s7_tcp_reconnect(fx):
    """TCP disconnect and reconnect of the MAVLink port while armed"""
    fx.settle(True)
    t_last = fx.link.last_aux_t
    fx.link.close()
    time.sleep(1.2)
    check_failsafe_delay(fx, "connection closed", t_last)
    check("servos held during the disconnect", fx.servos() == POSE, str(fx.servos()))
    try:
        link = fx.reconnect_link()
    except RuntimeError as exc:
        check("MAVLink port accepts a new connection", False, str(exc))
        return
    check("MAVLink port accepts a new connection", True)
    link.frames(0.3, False, RUN, POSE)
    check("disarmed frames after reconnect: motors min_command", fx.motors() == IDLE, str(fx.motors()))
    link.send_aux(True, RUN, POSE)
    check("armed frame after reconnect: motors follow", fx.wait_outputs(motors=RUN) is not None, str(fx.motors()))
    # SITL delivers heartbeats queued while no client was connected right after the connect: ignore the first 0.3 s
    t_connect = time.monotonic()
    link.frames(2.5, True, RUN, POSE)
    fresh = link.heartbeats_between(t_connect + 0.3, time.monotonic())
    check("fresh HEARTBEATs on the new connection report armed", len(fresh) >= 1 and all(a for _, a in fresh),
          "%d heartbeats" % len(fresh))


def s8_long_outage(fx):
    """Long outage (10 s) then recovery"""
    fx.settle(True)
    t_last = fx.link.last_aux_t
    time.sleep(10.0)
    t_end = time.monotonic()
    beats = fx.link.heartbeats_between(t_last + 0.6, t_end)
    check("HEARTBEAT kept running through the outage (8-11 in ~9.4 s)", 8 <= len(beats) <= 11, "%d heartbeats" % len(beats))
    check("all HEARTBEATs of the outage report disarmed", all(not armed for _, armed in beats))
    check("motors min_command after the outage", fx.motors() == IDLE, str(fx.motors()))
    fx.link.send_aux(False, RUN, [1100, 1200, 1300, 1400, 1500])
    check("board responsive: servos follow the first frame", fx.wait_outputs(servos=[1100, 1200, 1300, 1400, 1500]) is not None,
          str(fx.servos()))
    fx.link.frames(0.3, False, RUN, [1100, 1200, 1300, 1400, 1500])    # >= 5 valid frames: arm-edge check P1
    fx.link.send_aux(True, RUN, POSE)
    check("armed frame: motors follow", fx.wait_outputs(motors=RUN) is not None, str(fx.motors()))


SCENARIOS = [s1_short_gaps, s2_long_gaps, s3_recover_disarmed_first, s4_recover_armed, s5_flapping, s6_heartbeat_only,
             s7_tcp_reconnect, s8_long_outage]

if __name__ == "__main__":
    code = h.run_scenarios(h.parse_args(__doc__), SCENARIOS)
    if disarm_delays_s:
        print("failsafe delay over %d measurements: min %.0f ms, max %.0f ms"
              % (len(disarm_delays_s), min(disarm_delays_s) * 1e3, max(disarm_delays_s) * 1e3))
    sys.exit(code)
