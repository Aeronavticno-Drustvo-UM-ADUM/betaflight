#!/usr/bin/env python3
"""Servo driver on SITL: arming and safety rules of docs/arming_safety.md (sections 2-5, 7, 8).

  test/test_05_arming_safety.py [--build] [--keep] [--verbose]

Scenarios:
 1. Transitions DISARMED -> ARMED -> DISARMED, ARMED -> FAILSAFE -> ARMED / DISARMED: timing, HEARTBEAT system_status, STATUSTEXT
 2. FAILSAFE never times out: 60 s without frames, then an armed frame recovers
 3. Arm-edge rule: first frame after boot says ARMED (inside the boot grace) -> arms at once
 4. Arm-edge rule: first frame after a gap > timeout says ARMED -> arms at once, no checks (even with a recent CRC error)
 5. P1 LINK: < 5 valid frames, CRC error in the last 1 s; a frame with an unknown message id is not a CRC error
 6. P2 BOOT_GRACE: inside sd_boot_grace_ms after boot
 7. P3 MOTOR_CONFIG: motor_pwm_protocol = DISABLED
 8. P4 CONFIG_ACTIVE: CLI session open, MSP write in the last 2 s (P5 save pending: not testable on SITL)
 9. Config lock: CLI entry and MSP write commands refused while armed and in failsafe, allowed when disarmed
10. Motor output written every task cycle in DISARMED, ARMED and FAILSAFE ([motorrate] hook)
11. SYS_STATUS at 1 Hz, errors_comm on CRC errors, drop_rate_comm on MAVLink sequence gaps
12. Parser reset after 1 s of silence: truncated frame, 1.5 s silence, valid frame applied at once
13. CLI settings sd_link_timeout_ms / sd_boot_grace_ms exist, defaults, change the behaviour

Rule D8: the satellite never refuses an ARMED frame. Every readiness scenario (P1-P4) checks: while the cause lasts and the
board is DISARMED, HEARTBEAT UNINIT + exact text `NOT READY: <CODE>` (severity WARNING, at most 2 per second); an ARMED frame
during the cause still arms at once (< 50 ms); once the cause is gone, HEARTBEAT STANDBY and no new NOT READY text.
Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common import mavlink_gen as mav  # noqa: E402
from common.harness import RUN, POSE, IDLE, check  # noqa: E402

T_FS = h.FAILSAFE_TIMEOUT_S
UNKNOWN_MSGID = 12999             # not in the ADUM dialect
DISARM_MIN_S, DISARM_MAX_S = 0.495, 0.600
BOOT_GRACE_S = 2.0                 # sd_boot_grace_ms default
SEV_CRITICAL, SEV_WARNING = mav.MAV_SEVERITY_CRITICAL, mav.MAV_SEVERITY_WARNING
STANDBY, ACTIVE, CRITICAL = mav.MAV_STATE_STANDBY, mav.MAV_STATE_ACTIVE, mav.MAV_STATE_CRITICAL
UNINIT = mav.MAV_STATE_UNINIT
MOTOR_LATENCY_S = 0.05
RATE_MIN = 120                     # SITL runs the servo task at ~150 Hz (measured 146-151), the target 200 Hz is a bench item

measured = {"fs_delay": [], "not_ready_latency": [], "rate": {}}


# ---------------------------------------------------------------- helpers
def motors_seen(fx, since, until=float("inf")):
    return set().union(*(fx.sitl.seen("motor", i, since, until) for i in range(h.MOTORS)))


def frames(fx, seconds, armed):
    fx.link.frames(seconds, armed, RUN, POSE)


def wait_boot(fx, seconds=BOOT_GRACE_S + 0.4):
    """Disarmed frames until `seconds` after the SITL start (boot grace over)."""
    while fx.since_boot() < seconds:
        frames(fx, 0.1, False)


def keep_fn(fx, armed):
    return lambda: frames(fx, 0.1, armed)


def disarmed(fx, seconds=0.4):
    """Known DISARMED state with a healthy link: valid disarmed frames at 50 Hz."""
    frames(fx, seconds, False)


def arm_edge(fx, label, timeout=0.5):
    """One armed frame after disarmed frames; checks the motors follow it at once. Returns (ok, time of the frame)."""
    t = fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=timeout)
    ok = check("%s: motors follow the armed frame (< 50 ms)" % label, wait is not None and wait < MOTOR_LATENCY_S,
               "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
    frames(fx, 0.2, True)
    return ok, t


def max_per_second(times):
    return max((sum(1 for u in times if t <= u < t + 1.0) for t in times), default=0)


def hb_status(fx, armed, after=None):
    """system_status of the first HEARTBEAT after `after` while frames of `armed` keep coming (armed None: no frames)."""
    after = time.monotonic() if after is None else after
    keep = keep_fn(fx, armed) if armed is not None else None
    return fx.link.status_after(after, 2.5, keep)


def report_not_ready(fx, code, label, since):
    """Checks the not-ready report since `since`: HEARTBEAT UNINIT, STATUSTEXT 'NOT READY: <code>' (WARNING), only this reason."""
    beats = [status for t, _, status in list(fx.link.hb_full) if t >= since]
    first_status = beats[0] if beats else None
    check("%s: HEARTBEAT UNINIT (0)" % label, first_status == UNINIT, "statuses %s" % beats[:3])
    text = "NOT READY: " + code
    first = fx.link.wait_text(text, since, 0.3)
    found = fx.link.texts_since(since, "NOT READY")
    check("%s: STATUSTEXT '%s'" % (label, text), first is not None, "texts %s" % [x[2] for x in found[:3]])
    if first:
        measured["not_ready_latency"].append(first[0] - since)
        check("%s: severity WARNING (4)" % label, first[1] == SEV_WARNING, "severity %d" % first[1])
    check("%s: only this reason reported" % label, all(x[2] == text for x in found), str(sorted({x[2] for x in found})))
    check("%s: at most 2 NOT READY texts per second" % label, max_per_second([x[0] for x in found]) <= 2,
          "%d texts" % len(found))


def arm_despite(fx, label, check_motors=True):
    """ARMED frame while not ready arms at once (D8), then a disarmed frame. Returns the seconds since the SITL start at the edge."""
    boot_s = fx.since_boot()
    if check_motors:
        arm_edge(fx, "%s: ARMED frame despite not ready" % label)
    else:
        fx.link.send_aux(True, RUN, POSE)
        frames(fx, 0.2, True)
    status = hb_status(fx, True)
    check("%s: armed despite not ready (HEARTBEAT ACTIVE, SAFETY_ARMED)" % label,
          status == ACTIVE and fx.link.board_armed is True, "status %s armed %s" % (status, fx.link.board_armed))
    fx.link.send_aux(False, RUN, POSE)
    frames(fx, 0.3, False)
    return boot_s


def check_not_ready(fx, code, label, keep=None, window=1.2, check_motors=True):
    """Cause present, frames DISARMED for `window` s: not-ready report; then an ARMED frame arms at once. Returns the SITL uptime at the edge."""
    t0 = time.monotonic()
    keep = keep or keep_fn(fx, False)
    while time.monotonic() < t0 + window:
        keep()
    report_not_ready(fx, code, label, t0)
    return arm_despite(fx, label, check_motors)


def check_ready_again(fx, label, settle=0.4):
    """Cause gone: HEARTBEAT STANDBY and no new NOT READY text."""
    frames(fx, settle, False)
    t = time.monotonic()
    status = hb_status(fx, False, t)
    frames(fx, 0.3, False)
    texts = fx.link.texts_since(t, "NOT READY")
    check("%s: HEARTBEAT STANDBY once the cause is gone" % label, status == STANDBY, "status %s" % status)
    check("%s: no NOT READY text once the cause is gone" % label, not texts, str([x[2] for x in texts]))


def cli_open(fx, armed, seconds=1.2):
    """Opens the CLI on tcp 5761 and sends '#'; frames of `armed` keep the link alive (None: no frames).
    Returns (socket, text received)."""
    sock = h.connect(h.CLI_PORT, 5.0)
    sock.sendall(b"#")
    data, end = b"", time.monotonic() + seconds
    sock.settimeout(0.05)
    while time.monotonic() < end:
        if armed is None:
            time.sleep(0.1)
        else:
            frames(fx, 0.1, armed)
        try:
            data += sock.recv(4096)
        except OSError:
            pass
    return sock, data.decode(errors="replace")


def cli_leave(fx, sock):
    """`exit` (SITL may restart) and close; restarts SITL if the process ended. Returns True when it restarted."""
    try:
        sock.sendall(b"exit\r\n")
    except OSError:
        pass
    end = time.monotonic() + 2.0
    while time.monotonic() < end and fx.sitl.alive():
        time.sleep(0.1)
    sock.close()
    if not fx.sitl.alive():
        fx.restart()
        return True
    return False


def cli_set(fx, commands, save=True):
    """Runs CLI commands (board disarmed), `save` restarts SITL. Returns the replies (banner first)."""
    sock = h.connect(h.CLI_PORT, 5.0)
    sock.sendall(b"#")
    replies = [h.read_until(sock, b"# ", 3.0)]
    for cmd in commands:
        sock.sendall(cmd.encode() + b"\n")
        replies.append(h.read_until(sock, b"\n# ", 2.0))
    if save:
        sock.sendall(b"save\n")
        end = time.monotonic() + 4.0
        while time.monotonic() < end and fx.sitl.alive():
            time.sleep(0.1)
        sock.close()
        fx.restart()
    else:
        sock.close()
    return replies


def cli_get(*names):
    """`get` of several settings in one CLI session (a second session right after the first does not answer). Returns {name: reply}."""
    sock = h.connect(h.CLI_PORT, 5.0)
    sock.sendall(b"#")
    h.read_until(sock, b"# ", 3.0)
    replies = {}
    for name in names:
        sock.sendall(("get %s\n" % name).encode())
        replies[name] = h.read_until(sock, b"\n# ", 2.0)
    sock.close()
    return replies


MSP_STATUS, MSP_REBOOT, MSP_EEPROM_WRITE, MSP_SET_MOTOR = 101, 68, 250, 214


def msp_frame(cmd, payload=b""):
    csum = len(payload) ^ cmd
    for b in payload:
        csum ^= b
    return b"$M<" + bytes([len(payload), cmd]) + payload + bytes([csum])


def msp(fx, cmd, payload=b"", armed=None, wait=0.4):
    """One MSPv1 request on tcp 5761; frames of `armed` keep the link alive meanwhile. Returns the raw reply."""
    sock = h.connect(h.CLI_PORT, 5.0)
    sock.sendall(msp_frame(cmd, payload))
    data, end = b"", time.monotonic() + wait
    sock.settimeout(0.05)
    while time.monotonic() < end:
        if armed is None:
            time.sleep(0.05)
        else:
            frames(fx, 0.05, armed)
        try:
            data += sock.recv(4096)
        except OSError:
            pass
    sock.close()
    return data


def msp_ack(data):
    return b"$M>" in data


# ---------------------------------------------------------------- scenarios
def s1_transitions(fx):
    """Transitions DISARMED/ARMED/FAILSAFE: timing, HEARTBEAT system_status, STATUSTEXT"""
    wait_boot(fx)
    disarmed(fx)
    check("DISARMED: motors min_command", fx.motors() == IDLE, str(fx.motors()))
    status = hb_status(fx, False)
    check("DISARMED: HEARTBEAT STANDBY (3), SAFETY_ARMED clear", status == STANDBY and fx.link.board_armed is False,
          "status %s armed %s" % (status, fx.link.board_armed))

    t0 = time.monotonic()
    _, t_edge = arm_edge(fx, "DISARMED->ARMED")
    status = hb_status(fx, True, t_edge + 0.05)
    check("ARMED: HEARTBEAT ACTIVE (4), SAFETY_ARMED set", status == ACTIVE and fx.link.board_armed is True,
          "status %s armed %s" % (status, fx.link.board_armed))
    check("DISARMED->ARMED: no STATUSTEXT", not fx.link.texts_since(t0), str([x[2] for x in fx.link.texts_since(t0)]))

    t_dis = fx.link.send_aux(False, RUN, POSE)
    wait = fx.wait_outputs(motors=IDLE, timeout=0.5)
    check("ARMED->DISARMED: motors min_command at once (< 50 ms)", wait is not None and wait < MOTOR_LATENCY_S,
          "after %s ms" % (None if wait is None else round(wait * 1e3)))
    frames(fx, 0.3, False)
    status = hb_status(fx, False, t_dis + 0.05)
    check("ARMED->DISARMED: HEARTBEAT STANDBY, not armed", status == STANDBY and fx.link.board_armed is False,
          "status %s armed %s" % (status, fx.link.board_armed))

    # ARMED -> FAILSAFE
    frames(fx, 0.3, False)
    arm_edge(fx, "re-arm")
    frames(fx, 0.3, True)
    t_last = fx.link.last_aux_t
    fx.link.gap(0.9)
    times = fx.disarm_times(t_last)
    delays = [t - t_last for t in times if t is not None]
    measured["fs_delay"].extend(delays)
    check("ARMED->FAILSAFE: motors min_command 495-600 ms after the last frame",
          len(delays) == h.MOTORS and DISARM_MIN_S <= min(delays) and max(delays) <= DISARM_MAX_S,
          "%s ms" % [round(d * 1e3) for d in delays])
    lost = fx.link.texts_since(t_last, "FAILSAFE: link lost")
    check("ARMED->FAILSAFE: STATUSTEXT 'FAILSAFE: link lost', severity CRITICAL (2), exactly once",
          len(lost) == 1 and lost[0][1] == SEV_CRITICAL, str(lost))
    if lost:
        check("ARMED->FAILSAFE: text 0.45-0.75 s after the last frame", 0.45 <= lost[0][0] - t_last <= 0.75,
              "%.0f ms" % ((lost[0][0] - t_last) * 1e3))
    status = fx.link.status_after(t_last + 0.6, 2.0)
    check("FAILSAFE: HEARTBEAT CRITICAL (5), SAFETY_ARMED clear", status == CRITICAL and fx.link.board_armed is False,
          "status %s armed %s" % (status, fx.link.board_armed))
    changes = [fx.sitl.changes("servo", i, t_last) for i in range(h.SERVOS)]
    check("FAILSAFE: servos hold (no change)", all(not c for c in changes) and fx.servos() == POSE, str(changes))

    # FAILSAFE -> ARMED
    t_back = fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=0.5)
    check("FAILSAFE->ARMED: motors follow the armed frame at once (< 50 ms), no checks",
          wait is not None and wait < MOTOR_LATENCY_S, "after %s ms" % (None if wait is None else round(wait * 1e3)))
    frames(fx, 0.3, True)
    back = fx.link.texts_since(t_back, "FAILSAFE: link restored")
    check("FAILSAFE->ARMED: STATUSTEXT 'FAILSAFE: link restored', severity CRITICAL (2)",
          len(back) == 1 and back[0][1] == SEV_CRITICAL, str(back))
    check("FAILSAFE->ARMED: no NOT READY text", not fx.link.texts_since(t_back, "NOT READY"))
    status = hb_status(fx, True, t_back + 0.05)
    check("FAILSAFE->ARMED: HEARTBEAT ACTIVE, armed", status == ACTIVE and fx.link.board_armed is True,
          "status %s armed %s" % (status, fx.link.board_armed))

    # ARMED -> FAILSAFE -> DISARMED
    fx.link.gap(0.9)
    check("second FAILSAFE: motors min_command", fx.motors() == IDLE, str(fx.motors()))
    new_pose = [1100, 1900, 1500, 1400, 1300]
    t_back = fx.link.send_aux(False, RUN, new_pose)
    wait = fx.wait_outputs(servos=new_pose, timeout=0.5)
    check("FAILSAFE->DISARMED: servos follow the disarmed frame", wait is not None, str(fx.servos()))
    t1 = time.monotonic()
    fx.link.frames(0.3, False, RUN, new_pose)
    check("FAILSAFE->DISARMED: motors stay min_command", motors_seen(fx, t1) <= {h.MOTOR_MIN}, str(fx.motors()))
    status = fx.link.status_after(t_back + 0.05, 2.0, lambda: fx.link.frames(0.1, False, RUN, new_pose))
    check("FAILSAFE->DISARMED: HEARTBEAT STANDBY (3)", status == STANDBY, "status %s" % status)
    print("   info: STATUSTEXTs after FAILSAFE->DISARMED: %s" % [x[2] for x in fx.link.texts_since(t_back)])
    fx.link.frames(0.3, False, RUN, POSE)


def s2_failsafe_forever(fx):
    """FAILSAFE never times out: 60 s without frames, then an armed frame recovers"""
    disarmed(fx)
    arm_edge(fx, "arm")
    t_last = fx.link.last_aux_t
    fx.link.gap(0.8)
    t_fs = time.monotonic()
    time.sleep(max(0.0, 60.0 - (time.monotonic() - t_last)))
    t_end = time.monotonic()
    check("%.0f s after the last frame: motors min_command, no change since the failsafe" % (t_end - t_last),
          fx.motors() == IDLE and all(not fx.sitl.changes("motor", i, t_fs) for i in range(h.MOTORS)), str(fx.motors()))
    check("servos held for 60 s", all(not fx.sitl.changes("servo", i, t_last) for i in range(h.SERVOS)), str(fx.servos()))
    beats = [x for x in fx.link.hb_full if t_fs <= x[0] <= t_end]
    check("HEARTBEAT 1 Hz all along (55-62 in ~59 s), all CRITICAL and not armed",
          55 <= len(beats) <= 62 and all(s == CRITICAL and not a for _, a, s in beats),
          "%d beats, statuses %s" % (len(beats), sorted({s for _, _, s in beats})))
    texts = [x[2] for x in fx.link.texts_since(t_last)]
    check("'FAILSAFE: link lost' sent once, no other STATUSTEXT in 60 s", texts == ["FAILSAFE: link lost"], str(texts))
    t = fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=0.5)
    check("armed frame after 60 s: motors follow at once", wait is not None,
          "after %s ms" % (None if wait is None else round(wait * 1e3)))
    frames(fx, 0.3, True)
    check("'FAILSAFE: link restored' sent", len(fx.link.texts_since(t, "FAILSAFE: link restored")) == 1,
          str([x[2] for x in fx.link.texts_since(t)]))
    frames(fx, 0.3, False)


def s3_first_frame_after_boot(fx):
    """First frame after boot says ARMED (inside the boot grace): arms at once"""
    check("SITL restarted", fx.restart())
    t_edge = fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=1.0)
    boot_s = fx.since_boot()
    check("first frame ARMED: motors follow at once (< 100 ms)", wait is not None and wait < 0.1,
          "after %s ms" % (None if wait is None else round(wait * 1e3)))
    check("armed inside the boot grace (test condition: %.2f s < %.1f s after the SITL start)" % (boot_s, BOOT_GRACE_S),
          boot_s < BOOT_GRACE_S)
    frames(fx, 0.5, True)
    texts = [x[2] for x in fx.link.texts_since(t_edge)]
    check("armed at once: no NOT READY, no failsafe text", not texts, str(texts))
    status = hb_status(fx, True)
    check("HEARTBEAT ACTIVE, armed", status == ACTIVE and fx.link.board_armed is True, "status %s" % status)
    frames(fx, 0.3, False)


def s4_first_frame_after_gap(fx):
    """First frame after a gap > timeout says ARMED: arms at once, no checks"""
    wait_boot(fx)
    disarmed(fx, 0.5)
    fx.link.send(h.corrupt_crc(fx.link.gen.aux_channels(False, RUN, POSE)))      # CRC error: P1 not ready for 1 s
    time.sleep(T_FS + 0.2)                                                         # gap > timeout: link session over
    t_edge = fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=0.5)
    check("armed frame after a gap: motors follow at once (< 50 ms) although P1 is not ready",
          wait is not None and wait < MOTOR_LATENCY_S, "after %s ms" % (None if wait is None else round(wait * 1e3)))
    frames(fx, 0.3, True)
    texts = [x[2] for x in fx.link.texts_since(t_edge, "NOT READY")]
    check("armed frame after a gap: no NOT READY text while armed", not texts, str(texts))
    frames(fx, 0.3, False)


def wait_new_heartbeat(fx, timeout=2.0):
    """Waits (no frames sent) until a new HEARTBEAT arrives; returns its time or None."""
    count = len(fx.link.hb_full)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if len(fx.link.hb_full) > count:
            return fx.link.hb_full[-1][0]
        time.sleep(0.01)
    return None


def s5_p1_link_not_ready(fx):
    """P1 LINK: < 5 valid frames, CRC error in the last 1 s; unknown message id is not a CRC error"""
    wait_boot(fx)
    disarmed(fx, 0.3)
    time.sleep(T_FS + 0.3)                       # gap: new link session on the next frame
    t_beat = wait_new_heartbeat(fx)              # sync: next HEARTBEAT comes 1 s after this one
    check("P1: HEARTBEAT seen during the gap (sync)", t_beat is not None)
    t0 = time.monotonic()
    for _ in range(3):                           # 3 valid frames 0.4 s apart: the session is < 5 frames old
        fx.link.send_aux(False, RUN, POSE)
        time.sleep(0.4)
    status = fx.link.status_after(t0, 0.6)
    report_not_ready(fx, "LINK", "P1 (3 frames in the session)", t0)
    check("P1: HEARTBEAT UNINIT came with fewer than 5 frames", status == UNINIT, "status %s" % status)
    arm_despite(fx, "P1 (4th frame of the session)")
    check_ready_again(fx, "P1 after 5+ frames")

    def crc_error_and_frames():
        fx.link.send(h.corrupt_crc(fx.link.gen.aux_channels(False, RUN, POSE)))
        frames(fx, 0.2, False)
    check_not_ready(fx, "LINK", "P1 (CRC errors, last 0.1-0.2 s before the edge)", keep=crc_error_and_frames)
    check_ready_again(fx, "P1 after the CRC window", settle=1.2)

    # valid frame with an unknown message id: ignored, not a CRC error
    sys.modules[type(fx.link.gen).__module__].CRC_EXTRA.setdefault(UNKNOWN_MSGID, 77)   # frame is well-formed, id unknown to the firmware
    unknown = fx.link.gen._frame(UNKNOWN_MSGID, bytes(range(1, 9)))
    t = time.monotonic()
    for _ in range(30):
        fx.link.send(unknown)
        frames(fx, 0.05, False)
    status = hb_status(fx, False, t)
    frames(fx, 0.3, False)
    texts = fx.link.texts_since(t, "NOT READY")
    check("P1: unknown message id sent 30x: HEARTBEAT stays STANDBY", status == STANDBY, "status %s" % status)
    check("P1: unknown message id: no NOT READY text", not texts, str([x[2] for x in texts]))
    frames(fx, 0.3, False)


def s6_p2_boot_grace(fx):
    """P2 BOOT_GRACE: inside sd_boot_grace_ms after boot"""
    check("SITL restarted", fx.restart())
    frames(fx, 0.1, False)
    t0 = time.monotonic()
    while time.monotonic() < t0 + 1.5 and not [1 for t, _, _ in list(fx.link.hb_full) if t >= t0]:
        frames(fx, 0.05, False)                  # until the first HEARTBEAT, well inside the grace
    report_not_ready(fx, "BOOT_GRACE", "P2", t0)
    boot_s = arm_despite(fx, "P2")
    check("P2: the edge was inside the grace (test condition: %.2f s after the SITL start)" % boot_s,
          boot_s < BOOT_GRACE_S - 0.05, "%.2f s" % boot_s)
    while fx.since_boot() < BOOT_GRACE_S + 0.3:
        frames(fx, 0.1, False)
    check_ready_again(fx, "P2 after the grace")


def s7_p3_motor_config(fx):
    """P3 MOTOR_CONFIG: motor_pwm_protocol = DISABLED"""
    replies = cli_set(fx, ["set motor_pwm_protocol = DISABLED"])
    check("CLI: motor_pwm_protocol = DISABLED accepted", "Invalid" not in replies[1], replies[1].strip()[:80])
    wait_boot(fx)
    disarmed(fx, 0.5)
    check_not_ready(fx, "MOTOR_CONFIG", "P3", check_motors=False)    # motor output is disabled: no motor values to follow
    cli_set(fx, ["set motor_pwm_protocol = ONESHOT125"])
    wait_boot(fx)
    disarmed(fx, 0.5)
    check_ready_again(fx, "P3 restored")


def s8_p4_config_active(fx):
    """P4 CONFIG_ACTIVE: CLI session open, MSP write in the last 2 s (P5: not testable on SITL)"""
    wait_boot(fx)
    disarmed(fx, 0.5)
    sock, banner = cli_open(fx, False)
    check("CLI entry allowed while disarmed", "Entering CLI" in banner, repr(banner[:60]))
    frames(fx, 0.4, False)
    check_not_ready(fx, "CONFIG_ACTIVE", "P4 (CLI open)")
    restarted = cli_leave(fx, sock)
    print("   info: `exit` %s" % ("restarted SITL" if restarted else "left SITL running"))
    wait_boot(fx)
    disarmed(fx, 2.2)
    check_ready_again(fx, "P4 CLI closed")
    disarmed(fx, 0.5)                            # MSP write in the last 2 s
    reply = msp(fx, MSP_EEPROM_WRITE, armed=False, wait=0.2)
    check("MSP EEPROM write accepted while disarmed", msp_ack(reply), repr(reply[:20]))
    check_not_ready(fx, "CONFIG_ACTIVE", "P4 (MSP write)", window=1.0)
    disarmed(fx, 2.2)
    check_ready_again(fx, "P4 MSP quiet")


def s9_config_lock(fx):
    """Config lock: CLI entry and MSP write commands refused while armed and in failsafe, allowed when disarmed"""
    wait_boot(fx)
    disarmed(fx, 2.2)
    arm_edge(fx, "arm")

    def probe(state, armed):
        sock, text = cli_open(fx, armed, 1.0)
        check("%s: CLI entry refused" % state, "Entering CLI" not in text, repr(text[:80]))
        sock.close()
        for name, cmd, payload in (("EEPROM write", MSP_EEPROM_WRITE, b""), ("reboot", MSP_REBOOT, b""),
                                   ("set motor", MSP_SET_MOTOR, struct.pack("<8H", 1100, 1100, 1100, 1100, 0, 0, 0, 0))):
            reply = msp(fx, cmd, payload, armed=armed)
            check("%s: MSP %s refused (no ack)" % (state, name), not msp_ack(reply), repr(reply[:20]))
        reply = msp(fx, MSP_STATUS, armed=armed)
        check("%s: MSP status (read only) still answered" % state, msp_ack(reply), repr(reply[:20]))
        if armed is None:
            time.sleep(0.8)
        else:
            frames(fx, 0.8, armed)
        check("%s: SITL alive after the reboot request" % state, fx.sitl.alive())

    t0 = time.monotonic()
    probe("ARMED", True)
    seen = motors_seen(fx, t0)
    check("ARMED: motors never left the commanded values", seen == set(RUN), str(sorted(seen)))
    frames(fx, 0.2, True)
    t_last = fx.link.last_aux_t
    fx.link.gap(0.8)
    check("FAILSAFE reached", fx.link.texts_since(t_last, "FAILSAFE: link lost") != [])
    probe("FAILSAFE", None)
    check("FAILSAFE: motors min_command", fx.motors() == IDLE, str(fx.motors()))
    frames(fx, 0.4, False)                       # disarmed again: allowed
    reply = msp(fx, MSP_EEPROM_WRITE, armed=False, wait=0.3)
    check("DISARMED: MSP EEPROM write allowed", msp_ack(reply), repr(reply[:20]))
    sock, text = cli_open(fx, False, 1.0)
    check("DISARMED: CLI entry allowed", "Entering CLI" in text, repr(text[:60]))
    cli_leave(fx, sock)
    wait_boot(fx)
    disarmed(fx, 2.5)


def s10_motor_rate(fx):
    """Motor output written every task cycle in DISARMED, ARMED and FAILSAFE ([motorrate] hook)"""
    wait_boot(fx)

    def sample(state, armed):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.6:
            if armed is None:
                time.sleep(0.1)
            else:
                frames(fx, 0.1, armed)
        rates = fx.sitl.rates(t0 + 1.15)
        measured["rate"][state] = rates
        check("%s: motor writes 120-250 per second (SITL task ~150 Hz, 200 Hz on the bench)" % state,
              len(rates) >= 2 and all(RATE_MIN <= r <= 250 for r in rates), "writes/s %s" % rates)

    disarmed(fx, 0.5)
    sample("DISARMED", False)
    arm_edge(fx, "arm")
    sample("ARMED", True)
    fx.link.gap(0.8)
    sample("FAILSAFE", None)
    frames(fx, 0.4, False)


def s11_sys_status(fx):
    """SYS_STATUS at 1 Hz, errors_comm on CRC errors, drop_rate_comm on MAVLink sequence gaps"""
    wait_boot(fx)
    disarmed(fx, 0.5)
    t0 = time.monotonic()
    frames(fx, 5.2, False)
    beats = [t for t, _ in fx.link.sys if t >= t0]
    gaps = sorted(b - a for a, b in zip(beats, beats[1:]))
    check("SYS_STATUS 1 Hz (4-6 in 5.2 s, median interval 0.8-1.2 s)",
          4 <= len(beats) <= 6 and bool(gaps) and 0.8 <= gaps[len(gaps) // 2] <= 1.2,
          "%d messages, intervals %s" % (len(beats), [round(g, 2) for g in gaps]))

    base = fx.link.sys_after(time.monotonic(), 1.5, keep_fn(fx, False))
    for _ in range(10):
        fx.link.send(h.corrupt_crc(fx.link.gen.aux_channels(False, RUN, POSE)))
        frames(fx, 0.04, False)
    after = fx.link.sys_after(time.monotonic(), 1.5, keep_fn(fx, False))
    if base and after:
        check("errors_comm +10 after 10 CRC errors", after[1]["errors_comm"] - base[1]["errors_comm"] == 10,
              "%d -> %d" % (base[1]["errors_comm"], after[1]["errors_comm"]))
    else:
        check("errors_comm: SYS_STATUS received", False, "base %s after %s" % (base, after))

    frames(fx, 1.3, False)
    for i in range(100):                         # every 10th frame lost on the wire
        if i % 10 == 9:
            fx.link.gen.skip_seq()
        fx.link.send_aux(False, RUN, POSE)
        time.sleep(0.02)
    lossy = fx.link.sys_after(time.monotonic(), 1.5, keep_fn(fx, False))
    check("drop_rate_comm > 0 with every 10th frame lost", lossy is not None and lossy[1]["drop_rate_comm"] > 0,
          "drop_rate_comm %s" % (lossy and lossy[1]["drop_rate_comm"]))
    if lossy:
        print("   info: drop_rate_comm %d (0.01 %%), expected ~900 for 1 lost in 11" % lossy[1]["drop_rate_comm"])
    frames(fx, 1.2, False)
    clean = fx.link.sys_after(time.monotonic(), 1.5, keep_fn(fx, False))
    check("drop_rate_comm back to 0 after a clean second", clean is not None and clean[1]["drop_rate_comm"] == 0,
          "drop_rate_comm %s" % (clean and clean[1]["drop_rate_comm"]))


def s12_parser_reset(fx):
    """Parser reset after 1 s of silence: truncated frame, 1.5 s silence, valid frame applied at once"""
    wait_boot(fx)
    disarmed(fx, 0.5)
    new_pose = [1100, 1900, 1500, 1400, 1300]
    fx.link.send(fx.link.gen.aux_channels(False, RUN, new_pose)[:15])            # header and a part of the payload
    time.sleep(1.5)
    fx.link.send_aux(False, RUN, new_pose)
    wait = fx.wait_outputs(servos=new_pose, timeout=0.5)
    check("valid frame after a truncated frame + 1.5 s silence: applied at once (< 50 ms)", wait is not None and wait < 0.05,
          "after %s ms, servos %s" % (None if wait is None else round(wait * 1e3), fx.servos()))
    frames(fx, 0.3, False)
    check("SITL alive", fx.sitl.alive())


def s13_settings(fx):
    """CLI settings sd_link_timeout_ms / sd_boot_grace_ms: defaults, effect"""
    replies = cli_get("sd_link_timeout_ms", "sd_boot_grace_ms")
    reply = replies["sd_link_timeout_ms"]
    check("sd_link_timeout_ms exists, default 500", "sd_link_timeout_ms = 500" in reply, reply.strip()[:80])
    reply = replies["sd_boot_grace_ms"]
    check("sd_boot_grace_ms exists, default 2000", "sd_boot_grace_ms = 2000" in reply, reply.strip()[:80])
    fx.restart()                                 # the CLI session state is unknown: start clean
    replies = cli_set(fx, ["set sd_link_timeout_ms = 1000", "set sd_boot_grace_ms = 500"])
    check("CLI accepts both settings", all("Invalid" not in r for r in replies), "; ".join(r.strip()[:40] for r in replies[1:]))
    frames(fx, 0.3, False)
    boot_s = fx.since_boot()
    arm_edge(fx, "sd_boot_grace_ms=500: edge %.2f s after boot" % boot_s)
    check("test condition: edge after the 500 ms grace and inside the default 2 s", 0.5 < boot_s < BOOT_GRACE_S - 0.1,
          "%.2f s" % boot_s)
    frames(fx, 0.3, True)
    t0 = time.monotonic()
    fx.link.gap(0.8)
    frames(fx, 0.3, True)
    seen = motors_seen(fx, t0)
    check("sd_link_timeout_ms=1000: a 0.8 s gap does not trigger the failsafe", seen == set(RUN), str(sorted(seen)))
    t_last = fx.link.last_aux_t
    fx.link.gap(1.5)
    times = fx.disarm_times(t_last)
    delays = [t - t_last for t in times if t is not None]
    measured["fs_delay"].extend(delays)
    check("sd_link_timeout_ms=1000: failsafe 995-1100 ms after the last frame",
          len(delays) == h.MOTORS and 0.995 <= min(delays) and max(delays) <= 1.100, "%s ms" % [round(d * 1e3) for d in delays])
    frames(fx, 0.3, False)
    cli_set(fx, ["set sd_link_timeout_ms = 500", "set sd_boot_grace_ms = 2000"])
    replies = cli_get("sd_link_timeout_ms", "sd_boot_grace_ms")
    check("defaults restored", "sd_link_timeout_ms = 500" in replies["sd_link_timeout_ms"]
          and "sd_boot_grace_ms = 2000" in replies["sd_boot_grace_ms"], str([r.strip()[:50] for r in replies.values()]))
    fx.restart()


SCENARIOS = [s1_transitions, s2_failsafe_forever, s3_first_frame_after_boot, s4_first_frame_after_gap, s5_p1_link_not_ready,
             s6_p2_boot_grace, s7_p3_motor_config, s8_p4_config_active, s9_config_lock, s10_motor_rate, s11_sys_status,
             s12_parser_reset, s13_settings]

if __name__ == "__main__":
    code = h.run_scenarios(h.parse_args(__doc__), SCENARIOS)
    f = measured["fs_delay"]
    if f:
        print("failsafe delay over %d measurements: min %.0f ms, max %.0f ms" % (len(f), min(f) * 1e3, max(f) * 1e3))
    rl = measured["not_ready_latency"]
    if rl:
        print("not-ready STATUSTEXT latency over %d: min %.0f ms, max %.0f ms" % (len(rl), min(rl) * 1e3, max(rl) * 1e3))
    for state, rates in measured["rate"].items():
        print("motor writes/s %s: %s" % (state, rates))
    sys.exit(code)
