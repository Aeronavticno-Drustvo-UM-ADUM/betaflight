#!/usr/bin/env python3
"""Hardware test 04: garbage, unknown ids and CRC errors (arming_safety.md sections 4, 5, 12).

Phases:
  1. Garbage bytes (no MAVLink start byte) between disarmed frames: readiness unchanged (skipped, not counted).
  2. 30 well-formed frames with an unknown message id: errors_comm +30, readiness unchanged.
  3. One AUX frame with a corrupted CRC: NOT READY (LINK), then ready again.
  4. All three error kinds while ARMED: the board stays ARMED, motors keep following the frame, clean disarm.
"""
import argparse
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import report as report_module  # noqa: E402
from common import sat_bench as bench  # noqa: E402
from test_hw_02_readiness import (  # noqa: E402  (streamer with timestamped HEARTBEAT/STATUSTEXT, board reset helpers)
    BOOT_WAIT_S, FRAME_PERIOD_S, MAV_STATE_STANDBY, NOT_READY_PREFIX, SWD_SETTLE_S,
    ReadinessStreamer, open_silent_streamer, reset_satellite)

ARMS = True   # phase 4 arms the board: the runner requires --props-off

UNKNOWN_MSGID = 12999                 # not in the ADUM dialect
UNKNOWN_FRAME_COUNT = 30
UNKNOWN_CRC_EXTRA = 77                # lets the generator build a well-formed frame of the unknown id
MAVLINK_START_BYTES = (0xFE, 0xFD)    # garbage never contains them: it must not start a frame
GARBAGE_SEED = 4
GARBAGE_BURST_MIN_BYTES = 5
GARBAGE_BURST_MAX_BYTES = 40
GARBAGE_DURATION_S = 4.0
GARBAGE_BURST_PERIOD_S = 0.05
ARMED_ERROR_DURATION_S = 1.5          # garbage / unknown-id injection time per step while armed
ARMED_MOTOR_US = 1300
MOTOR_TOLERANCE_US = 3.0              # OneShot125 pulse tolerance, same order as the output check
STANDBY_MAX_S = 3.0                   # reset + disarmed frames -> STANDBY
NO_TEXT_GRACE_S = 0.2
HEARTBEAT_SYNC_TIMEOUT_S = 3.0
CRC_INJECT_AFTER_HEARTBEAT_S = 0.65   # so the next HEARTBEAT (1 Hz) falls inside the 1 s CRC quiet window
CRC_OBSERVE_S = 4.0
RECOVERY_MAX_S = 3.0                  # CRC error -> STANDBY again (1 s quiet + 5 frames + HEARTBEAT granularity 1 s)
SYS_STATUS_WAIT_S = 1.5               # SYS_STATUS rate is not fixed: wait this long for a fresh one
ARM_REPORT_MAX_S = 1.5
DISARM_REPORT_MAX_S = 1.5
DISARMED_FRAMES_BEFORE_ARM_S = 0.3


class RecordingParser:
    """Wraps the frame parser and records every decoded SYS_STATUS (errors_comm) with its time."""

    def __init__(self, inner_parser):
        self.inner_parser = inner_parser
        self.sys_status = []     # (time, decoded dict)

    def feed(self, data):
        """Parses like the inner parser, side effect: SYS_STATUS frames are recorded."""
        frames = self.inner_parser.feed(data)
        for message_id, payload in frames:
            if message_id == bench.hw.mav.MSG_SYS_STATUS:
                self.sys_status.append((time.monotonic(), bench.hw.mav.decode_sys_status(payload)))
        return frames


def open_error_streamer():
    """Silent (paused) streamer whose parser also records SYS_STATUS."""
    streamer = open_silent_streamer()
    streamer.parser = RecordingParser(streamer.parser)
    return streamer


def write_bytes(streamer, data):
    """Sends raw bytes on the master link (between the streamer's own frames)."""
    bench.serial_io.write_all(streamer.fd, data)


def make_garbage_burst(randomizer):
    """Random bytes without a MAVLink start byte."""
    length = randomizer.randint(GARBAGE_BURST_MIN_BYTES, GARBAGE_BURST_MAX_BYTES)
    burst = bytearray()
    while len(burst) < length:
        value = randomizer.randrange(256)
        if value not in MAVLINK_START_BYTES:
            burst.append(value)
    return bytes(burst)


def inject_garbage(streamer, duration_s, randomizer):
    """Sends garbage bursts for duration_s; returns the number of bursts."""
    bursts = 0
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        write_bytes(streamer, make_garbage_burst(randomizer))
        bursts += 1
        time.sleep(GARBAGE_BURST_PERIOD_S)
    return bursts


def make_unknown_frame(streamer):
    """Well-formed MAVLink 2 frame with a message id the firmware does not know."""
    generator = streamer.generator
    sys.modules[type(generator).__module__].CRC_EXTRA.setdefault(UNKNOWN_MSGID, UNKNOWN_CRC_EXTRA)
    return generator._frame(UNKNOWN_MSGID, bytes(range(1, 9)))


def inject_unknown_frames(streamer, count):
    """Sends count unknown-id frames, one per frame period."""
    frame = make_unknown_frame(streamer)
    for _ in range(count):
        write_bytes(streamer, frame)
        time.sleep(FRAME_PERIOD_S)


def make_bad_crc_aux_frame(streamer):
    """ADUM_AUX_CHANNELS frame (disarmed state of the streamer) with the last CRC byte flipped."""
    frame = streamer.generator.aux_channels(streamer.armed, list(streamer.motors), list(streamer.servos))
    return frame[:-1] + bytes([frame[-1] ^ 0xFF])


def not_ready_texts_since(streamer, since):
    """NOT READY STATUSTEXTs received after `since`."""
    return [text for time_, text in list(streamer.texts) if time_ > since and text.startswith(NOT_READY_PREFIX)]


def statuses_since(streamer, since):
    """HEARTBEAT system_status values received after `since`."""
    return [status for time_, status in list(streamer.heartbeats) if time_ > since]


def latest_errors_comm(streamer):
    """errors_comm of the newest SYS_STATUS, None when none was received."""
    records = streamer.parser.sys_status
    return records[-1][1]["errors_comm"] if records else None


def wait_fresh_errors_comm(streamer):
    """errors_comm of the first SYS_STATUS received from now on (None on timeout)."""
    start = time.monotonic()
    def fresh_value():
        for time_, decoded in list(streamer.parser.sys_status):
            if time_ > start:
                return (decoded["errors_comm"],)     # tuple: a value of 0 stays truthy for wait_until
        return None
    found = bench.wait_until(fresh_value, SYS_STATUS_WAIT_S)
    return None if found is None else found[0]


def check_readiness_unchanged(report, streamer, since, label):
    """HEARTBEAT stays STANDBY and no NOT READY text since `since`."""
    statuses = statuses_since(streamer, since)
    texts = not_ready_texts_since(streamer, since + NO_TEXT_GRACE_S)
    report.check("%s: HEARTBEAT stays STANDBY" % label,
                 statuses and all(status == MAV_STATE_STANDBY for status in statuses),
                 "statuses=%s" % sorted(set(statuses)))
    report.check("%s: no NOT READY text" % label, not texts, "texts=%s" % texts if texts else "none")


def reach_standby(report, streamer):
    """Reset the board and stream disarmed frames until HEARTBEAT STANDBY; returns True on success."""
    streamer.paused = True
    streamer.armed = False
    reset_satellite()
    time.sleep(BOOT_WAIT_S)
    streamer.clear_records()
    streamer.paused = False

    def standby_seen():
        return any(status == MAV_STATE_STANDBY for _time, status in list(streamer.heartbeats))

    reached = bench.wait_until(standby_seen, STANDBY_MAX_S) is not None
    report.check("STANDBY reached with disarmed frames", reached)
    return reached


def phase_garbage(report, streamer):
    """Phase 1: garbage between disarmed frames."""
    report.start_phase("1", "garbage bytes between disarmed frames")
    if not reach_standby(report, streamer):
        return False
    time.sleep(SWD_SETTLE_S)
    before = bench.diag()
    start = time.monotonic()
    bursts = inject_garbage(streamer, GARBAGE_DURATION_S, random.Random(GARBAGE_SEED))
    time.sleep(SWD_SETTLE_S)
    after = bench.diag()
    added = after["parseErrors"] - before["parseErrors"]
    report.number("garbage_bursts_sent", bursts, "bursts")
    # bytes without a start byte never begin a frame: the parser skips them, nothing to count (info only)
    report.number("parseErrors_added_by_garbage", added, "errors")
    check_readiness_unchanged(report, streamer, start, "garbage")
    return True


def phase_unknown_id(report, streamer):
    """Phase 2: 30 unknown-id frames."""
    report.start_phase("2", "30 frames with an unknown message id")
    time.sleep(SWD_SETTLE_S)
    before = bench.diag()
    errors_comm_before = wait_fresh_errors_comm(streamer)
    start = time.monotonic()
    inject_unknown_frames(streamer, UNKNOWN_FRAME_COUNT)
    time.sleep(SWD_SETTLE_S)
    after = bench.diag()
    errors_comm_after = wait_fresh_errors_comm(streamer)
    added = after["parseErrors"] - before["parseErrors"]
    report.check("parseErrors +%d (SWD)" % UNKNOWN_FRAME_COUNT, added == UNKNOWN_FRAME_COUNT,
                 "before=%d after=%d" % (before["parseErrors"], after["parseErrors"]))
    if errors_comm_before is None or errors_comm_after is None:
        report.check("SYS_STATUS errors_comm received", False,
                     "before=%s after=%s" % (errors_comm_before, errors_comm_after))
    else:
        report.check("SYS_STATUS errors_comm +%d" % UNKNOWN_FRAME_COUNT,
                     errors_comm_after - errors_comm_before == UNKNOWN_FRAME_COUNT,
                     "%d -> %d" % (errors_comm_before, errors_comm_after))
    check_readiness_unchanged(report, streamer, start, "unknown id")


def phase_crc_error(report, streamer):
    """Phase 3: one corrupted-CRC AUX frame: NOT READY (LINK), then STANDBY again."""
    report.start_phase("3", "one AUX frame with a bad CRC")
    last_heartbeat = streamer.heartbeat_time
    new_heartbeat = bench.wait_until(lambda: streamer.heartbeat_time > last_heartbeat, HEARTBEAT_SYNC_TIMEOUT_S)
    report.check("HEARTBEAT seen (sync)", new_heartbeat is not None)
    if new_heartbeat is None:
        return
    time.sleep(CRC_INJECT_AFTER_HEARTBEAT_S)
    injected_at = time.monotonic()
    write_bytes(streamer, make_bad_crc_aux_frame(streamer))
    time.sleep(CRC_OBSERVE_S)

    statuses = statuses_since(streamer, injected_at)
    texts = not_ready_texts_since(streamer, injected_at)
    not_standby = [status for status in statuses if status != MAV_STATE_STANDBY]
    link_texts = [text for text in texts if text == "NOT READY: LINK"]
    report.check("NOT READY reported (HEARTBEAT not STANDBY and/or NOT READY: LINK)",
                 bool(not_standby) or bool(link_texts),
                 "statuses=%s texts=%s" % (statuses, texts))
    report.check("NOT READY reason is LINK", bool(link_texts) or not texts, "texts=%s" % texts)

    recovered_times = [time_ for time_, status in list(streamer.heartbeats)
                       if time_ > injected_at and status == MAV_STATE_STANDBY]
    not_ready_times = [time_ for time_, status in list(streamer.heartbeats)
                       if time_ > injected_at and status != MAV_STATE_STANDBY]
    after_not_ready = [time_ for time_ in recovered_times if not not_ready_times or time_ > max(not_ready_times)]
    recovery_s = (after_not_ready[0] - injected_at) if after_not_ready else None
    report.check("STANDBY again within %.1f s" % RECOVERY_MAX_S,
                 recovery_s is not None and recovery_s <= RECOVERY_MAX_S,
                 "recovery=%s s" % (None if recovery_s is None else round(recovery_s, 2)))
    if recovery_s is not None:
        report.number("crc_error_recovery_time", recovery_s, "s (first STANDBY HEARTBEAT, 1 s granularity)")


def check_motors_follow(report, label):
    """Motor pulses (OneShot125 = us / divider) match the commanded value."""
    pulses = bench.motor_pulses_us()
    wanted_us = ARMED_MOTOR_US / bench.hw.ONESHOT125_DIVIDER
    worst = max(abs(pulse - wanted_us) for pulse in pulses.values())
    report.check("%s: motor pulses follow %d (OneShot125 %.1f us +-%.0f)" % (
        label, ARMED_MOTOR_US, wanted_us, MOTOR_TOLERANCE_US), worst <= MOTOR_TOLERANCE_US,
        "worst=%.1f us pulses=%s" % (worst, {pin: round(pulse, 1) for pin, pulse in pulses.items()}))


def check_still_armed(report, streamer, baseline, label):
    """SWD state ARMED, no state change since the baseline, no FAILSAFE."""
    time.sleep(SWD_SETTLE_S)
    current = bench.diag()
    report.check("%s: SWD state ARMED, no state change" % label,
                 current["lastToState"] == bench.STATE_ARMED and current["stateChanges"] == baseline["stateChanges"],
                 "lastToState=%d stateChanges %d -> %d" % (
                     current["lastToState"], baseline["stateChanges"], current["stateChanges"]))
    report.check("%s: not FAILSAFE" % label, current["lastToState"] != bench.STATE_FAILSAFE)
    check_motors_follow(report, label)


def phase_errors_while_armed(report, streamer):
    """Phase 4: all error kinds while ARMED change nothing; then a clean disarm."""
    report.start_phase("4", "garbage, unknown id and bad CRC while ARMED")
    streamer.motors = [ARMED_MOTOR_US] * bench.hw.MOTOR_COUNT
    time.sleep(DISARMED_FRAMES_BEFORE_ARM_S)
    arm_sent_at = time.monotonic()
    streamer.armed = True
    armed_after = streamer.wait_board_state(True, ARM_REPORT_MAX_S, since=arm_sent_at)
    report.check("board arms (HEARTBEAT armed)", armed_after is not None)
    if armed_after is None:
        streamer.armed = False
        return
    time.sleep(SWD_SETTLE_S)
    baseline = bench.diag()
    check_still_armed(report, streamer, baseline, "baseline")

    errors_before = baseline["parseErrors"]
    inject_garbage(streamer, ARMED_ERROR_DURATION_S, random.Random(GARBAGE_SEED + 1))
    check_still_armed(report, streamer, baseline, "garbage")

    inject_unknown_frames(streamer, UNKNOWN_FRAME_COUNT)
    check_still_armed(report, streamer, baseline, "unknown id")

    write_bytes(streamer, make_bad_crc_aux_frame(streamer))
    check_still_armed(report, streamer, baseline, "bad CRC")
    errors_after = bench.diag()["parseErrors"]
    report.check("errors were counted while armed", errors_after > errors_before,
                 "parseErrors %d -> %d" % (errors_before, errors_after))

    streamer.armed = False
    disarm_sent_at = time.monotonic()
    disarmed_after = streamer.wait_board_state(False, DISARM_REPORT_MAX_S, since=disarm_sent_at)
    report.check("clean disarm (HEARTBEAT disarmed)", disarmed_after is not None)
    time.sleep(SWD_SETTLE_S)
    disarmed = bench.diag()
    report.check("state DISARMED over SWD", disarmed["lastToState"] == bench.STATE_DISARMED,
                 "lastToState=%d" % disarmed["lastToState"])


def main():
    """Runs the four phases and writes summary.json."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    arguments = parser.parse_args()

    report = report_module.Report("test_hw_04_error_frames", arguments.run_dir)
    ok, problems = bench.preflight()
    if not ok:
        report.setup_failed = True
        for problem in problems:
            report.check("preflight: " + problem, False)
        return report.finish()

    streamer = open_error_streamer()
    try:
        if phase_garbage(report, streamer):
            phase_unknown_id(report, streamer)
            phase_crc_error(report, streamer)
            phase_errors_while_armed(report, streamer)
    finally:
        streamer.armed = False           # always leave the board disarmed
        streamer.motors = [bench.hw.mav.PULSE_IGNORE] * bench.hw.MOTOR_COUNT
        time.sleep(0.2)
        streamer.stop()
    return report.finish()


if __name__ == "__main__":
    sys.exit(main())
