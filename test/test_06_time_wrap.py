#!/usr/bin/env python3
"""Servo driver on SITL: fix W, timestamps older than 35.8 min (docs/fix_W_timestamps.md).

  test/test_06_time_wrap.py [--build] [--keep] [--verbose]

Uses the SITL-only time-jump hook (+25 min of SITL clock per jump). Two jumps, each followed by >= 2 s of normal running,
make old timestamps ~50 min old: inside the int32 negative window (35.8-71.6 min) of the old `cmpTimeUs()` code.

Scenarios (each starts with a SITL restart, so the clock is fresh):
 1. Site 1: CRC error, 2 jumps, disarmed frames, arm edge -> ARMS, no ARM REJECTED
 2. Site 3: rejected edge gives the text, 2 jumps, a second rejected edge gives the text again (rate limit not stuck)
 3. Site 2: config activity (MSP EEPROM write; CLI `exit` restarts SITL and would reset the clock), 2 jumps, arm edge -> ARMS
 4. HEARTBEAT keeps arriving at ~1 Hz after each jump

Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common.harness import RUN, POSE, check  # noqa: E402

BOOT_GRACE_S = 2.0                 # sd_boot_grace_ms default
MOTOR_LATENCY_S = 0.05
RUNNING_BETWEEN_JUMPS_S = 2.5      # normal running after each jump (>= 2 s)
MSP_EEPROM_WRITE = 250


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


def rejection_texts(fx, since):
    """Texts 'ARM REJECTED...' received since `since`."""
    return [x[2] for x in fx.link.texts_since(since, "ARM REJECTED")]


def arm_edge_arms(fx, label):
    """One armed frame: checks the motors follow at once and no rejection text came."""
    t_edge = fx.link.send_aux(True, RUN, POSE)
    wait = fx.wait_outputs(motors=RUN, timeout=0.5)
    check("%s: motors follow the armed frame (< 50 ms)" % label, wait is not None and wait < MOTOR_LATENCY_S,
          "after %s ms, motors %s" % (None if wait is None else round(wait * 1e3), fx.motors()))
    frames(fx, 0.3, True)
    texts = rejection_texts(fx, t_edge)
    check("%s: no ARM REJECTED text" % label, not texts, str(texts))
    frames(fx, 0.3, False)


def rejected_edge_after_crc(fx, label):
    """CRC error, then an arm edge 0.2 s later (LINK_NOT_READY). Checks the rejection text. Then disarms."""
    send_crc_error(fx)
    frames(fx, 0.2, False)
    t_edge = fx.link.send_aux(True, RUN, POSE)
    frames(fx, 1.0, True)
    texts = rejection_texts(fx, t_edge)
    check("%s: ARM REJECTED text" % label, "ARM REJECTED: LINK_NOT_READY" in texts, str(texts[:3]))
    frames(fx, 0.5, False)


# ---------------------------------------------------------------- scenarios
def s1_crc_window(fx):
    """Site 1: CRC error, 2 jumps, arm edge arms"""
    fresh_board(fx)
    send_crc_error(fx)
    frames(fx, 0.3, False)                       # the CRC error must be handled before the first jump
    two_jumps(fx)
    frames(fx, 0.4, False)
    arm_edge_arms(fx, "after CRC error + 2 jumps")


def s2_reject_text_rate_limit(fx):
    """Site 3: rejection text rate limit after 2 jumps"""
    fresh_board(fx)
    rejected_edge_after_crc(fx, "before the jumps")
    two_jumps(fx)
    frames(fx, 0.4, False)
    rejected_edge_after_crc(fx, "after 2 jumps")


def s3_config_activity(fx):
    """Site 2: config activity, 2 jumps, arm edge arms"""
    fresh_board(fx)
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
    check("MSP EEPROM write accepted while disarmed", b"$M>" in reply, repr(reply[:20]))
    two_jumps(fx)
    frames(fx, 0.4, False)
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


SCENARIOS = [s1_crc_window, s2_reject_text_rate_limit, s3_config_activity, s4_heartbeat_after_jump]

if __name__ == "__main__":
    sys.exit(h.run_scenarios(h.parse_args(__doc__), SCENARIOS))
