#!/usr/bin/env python3
"""Servo driver on SITL: fix W, timestamps older than 35.8 min (docs/fix_W_timestamps.md).

  test/test_06_time_wrap.py [--build] [--keep] [--verbose]

Uses the SITL-only time-jump hook (+25 min of SITL clock per jump). Two jumps, each followed by >= 2 s of normal running,
make old timestamps ~50 min old: inside the int32 negative window (35.8-71.6 min) of the old `cmpTimeUs()` code.

Scenarios (each starts with a SITL restart, so the clock is fresh):
 1. Site 1: CRC error, 2 jumps, HEARTBEAT back to STANDBY without NOT READY text (CRC window not stuck), arm edge arms
 2. Site 3: CRC error -> NOT READY: LINK + UNINIT, 2 jumps, new CRC error -> NOT READY: LINK again, clears to STANDBY
 3. Site 2: config activity (MSP EEPROM write; CLI `exit` restarts SITL and would reset the clock) -> NOT READY: CONFIG_ACTIVE
    before and after 2 jumps, clears to STANDBY, arm edge arms
 4. HEARTBEAT keeps arriving at ~1 Hz after each jump

Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common import mavlink_gen as mav  # noqa: E402
from common.harness import RUN, POSE, check  # noqa: E402

BOOT_GRACE_S = 2.0                 # sd_boot_grace_ms default
MOTOR_LATENCY_S = 0.05
RUNNING_BETWEEN_JUMPS_S = 2.5      # normal running after each jump (>= 2 s)
MSP_EEPROM_WRITE = 250
UNINIT = mav.MAV_STATE_UNINIT
STANDBY = mav.MAV_STATE_STANDBY
ACTIVE = mav.MAV_STATE_ACTIVE


# ---------------------------------------------------------------- helpers
def frames(fx, seconds, armed):
    """Valid AUX frames of `armed` at 50 Hz for `seconds`."""
    fx.link.frames(seconds, armed, RUN, POSE)


def fresh_board(fx):
    """Restarts SITL (fresh clock), waits for the boot grace, healthy disarmed link."""
    fx.restart()
    while fx.since_boot() < BOOT_GRACE_S + 0.4:
        frames(fx, 0.1, False)
    frames(fx, 0.5, False)


def two_jumps(fx, armed=False):
    """Two time jumps (+50 min), each followed by normal running with valid frames."""
    for _ in range(2):
        fx.time_jump()
        frames(fx, RUNNING_BETWEEN_JUMPS_S, armed)


def send_crc_error(fx):
    """One AUX frame with a broken CRC."""
    fx.link.send(h.corrupt_crc(fx.link.gen.aux_channels(False, RUN, POSE)))


def report_not_ready(fx, code, label, since):
    """Checks the not-ready report since `since`: HEARTBEAT UNINIT and STATUSTEXT 'NOT READY: <code>'."""
    frames(fx, 1.5, False)                       # a HEARTBEAT (1 Hz) falls inside the 1 s window
    beats = [status for t, _, status in list(fx.link.hb_full) if t >= since]
    check("%s: HEARTBEAT UNINIT (0) seen" % label, UNINIT in beats, "statuses %s" % beats[:3])
    text = "NOT READY: " + code
    found = [x[2] for x in fx.link.texts_since(since, "NOT READY")]
    check("%s: STATUSTEXT '%s'" % (label, text), text in found, "texts %s" % sorted(set(found)))


def check_ready_again(fx, label, settle=0.4):
    """Cause gone: HEARTBEAT STANDBY and no new NOT READY text."""
    frames(fx, settle, False)
    t = time.monotonic()
    status = fx.link.status_after(t, 2.5, lambda: frames(fx, 0.1, False))
    frames(fx, 0.3, False)
    texts = fx.link.texts_since(t, "NOT READY")
    check("%s: HEARTBEAT STANDBY (window not stuck)" % label, status == STANDBY, "status %s" % status)
    check("%s: no NOT READY text" % label, not texts, str([x[2] for x in texts]))


def arm_edge_arms(fx, label):
    """One armed frame (D8: never refused): checks the motors follow at once and the board reports armed."""
    fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=0.5)
    check("%s: motors follow the armed frame (< 50 ms)" % label, wait is not None and wait < MOTOR_LATENCY_S,
          "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
    frames(fx, 0.3, True)
    status = fx.link.status_after(time.monotonic(), 2.5, lambda: frames(fx, 0.1, True))
    check("%s: HEARTBEAT ACTIVE, SAFETY_ARMED" % label, status == ACTIVE and fx.link.board_armed is True,
          "status %s armed %s" % (status, fx.link.board_armed))
    frames(fx, 0.3, False)


def msp_eeprom_write(fx):
    """MSP EEPROM write on the CLI port (config activity) while disarmed frames keep coming. Returns True if accepted."""
    reply = b""
    sock = h.connect(h.CLI_PORT, 5.0)
    payload_length, command = 0, MSP_EEPROM_WRITE
    sock.sendall(b"$M<" + bytes([payload_length, command, payload_length ^ command]))
    sock.settimeout(0.05)
    end = time.monotonic() + 0.3
    while time.monotonic() < end:
        frames(fx, 0.05, False)
        try:
            reply += sock.recv(4096)
        except OSError:
            pass
    sock.close()
    return b"$M>" in reply


# ---------------------------------------------------------------- scenarios
def s1_crc_window(fx):
    """Site 1: CRC error, 2 jumps, STANDBY again (CRC window not stuck), arm edge arms"""
    fresh_board(fx)
    send_crc_error(fx)
    frames(fx, 0.3, False)                       # the CRC error must be handled before the first jump
    two_jumps(fx)
    frames(fx, 0.4, False)
    check_ready_again(fx, "after CRC error + 2 jumps")
    arm_edge_arms(fx, "after CRC error + 2 jumps")


def s2_not_ready_link_after_jumps(fx):
    """Site 3: NOT READY: LINK before and after 2 jumps (window re-arms) and clears within ~2 s"""
    fresh_board(fx)
    t_crc = time.monotonic()
    send_crc_error(fx)
    report_not_ready(fx, "LINK", "before the jumps", t_crc)
    check_ready_again(fx, "before the jumps", settle=0.8)
    two_jumps(fx)
    frames(fx, 0.4, False)
    t_crc = time.monotonic()
    send_crc_error(fx)
    report_not_ready(fx, "LINK", "after 2 jumps", t_crc)
    check_ready_again(fx, "after 2 jumps", settle=0.8)


def s3_config_activity(fx):
    """Site 2: NOT READY: CONFIG_ACTIVE before and after 2 jumps, clears after the window, arm edge arms"""
    fresh_board(fx)
    t_write = time.monotonic()
    accepted = msp_eeprom_write(fx)
    check("MSP EEPROM write accepted while disarmed", accepted, "no $M> reply")
    report_not_ready(fx, "CONFIG_ACTIVE", "before the jumps", t_write)
    frames(fx, 2.0, False)
    check_ready_again(fx, "before the jumps")
    two_jumps(fx)
    frames(fx, 0.4, False)
    t_write = time.monotonic()
    accepted = msp_eeprom_write(fx)
    check("MSP EEPROM write accepted after the jumps", accepted, "no $M> reply")
    report_not_ready(fx, "CONFIG_ACTIVE", "after 2 jumps", t_write)
    frames(fx, 2.0, False)
    check_ready_again(fx, "after 2 jumps")
    arm_edge_arms(fx, "after config activity + 2 jumps")


def s4_heartbeat_after_jump(fx):
    """HEARTBEAT at ~1 Hz after each jump"""
    fresh_board(fx)
    for jump in (1, 2):
        fx.time_jump()
        begin = time.monotonic()
        frames(fx, 3.5, False)
        count = len(fx.link.heartbeats_between(begin, time.monotonic()))
        check("jump %d: HEARTBEAT keeps arriving (~1 Hz, 3.5 s window)" % jump, 2 <= count <= 5, "%d heartbeats" % count)


SCENARIOS = [s1_crc_window, s2_not_ready_link_after_jumps, s3_config_activity, s4_heartbeat_after_jump]

if __name__ == "__main__":
    sys.exit(h.run_scenarios(h.parse_args(__doc__), SCENARIOS))
