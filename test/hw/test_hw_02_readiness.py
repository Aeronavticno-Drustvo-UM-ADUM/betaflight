#!/usr/bin/env python3
"""Hardware test 02: satellite readiness reporting (D8, arming_safety.md section 11).

Phases:
  1. Reset over SWD, FTDI silent: HEARTBEAT UNINIT and STATUSTEXT "NOT READY: LINK/BOOT_GRACE" at ~1 Hz.
  2. Disarmed AUX frames: HEARTBEAT STANDBY within ~1.5 s, NOT READY texts stop.
  3. Reset, 2 disarmed frames, then ARMED (arm edge while not ready): the board arms anyway, then disarm.
"""
import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import report as report_module  # noqa: E402
from common import sat_bench as bench  # noqa: E402

ARMS = True   # phase 3 arms the board: the runner requires --props-off

MAV_STATE_UNINIT = 0
MAV_STATE_STANDBY = 3
CAUSE_ARMED_FRAME = 2
CAUSE_DISARM_FRAME = 3
REASON_LINK = 0

NOT_READY_PREFIX = "NOT READY:"
ACCEPTED_NOT_READY_TEXTS = ("NOT READY: LINK", "NOT READY: BOOT_GRACE")

BOOT_WAIT_S = 2.0               # board start-up after the SWD reset: UART1 receives only ~1.0-1.5 s after it
SILENT_OBSERVE_S = 4.0          # phase 1 observation time
STANDBY_MAX_S = 1.5             # disarmed frames -> STANDBY
NO_TEXT_OBSERVE_S = 3.0         # observation after STANDBY, no NOT READY text allowed
TEXT_GRACE_S = 0.2              # a text sent just before the STANDBY heartbeat may still arrive after it
HEARTBEAT_PERIOD_MIN_S = 0.5    # accepted 1 Hz band
HEARTBEAT_PERIOD_MAX_S = 1.5
ARM_REPORT_MAX_S = 1.5          # armed frame -> HEARTBEAT ARMED
DISARM_REPORT_MAX_S = 1.5
SWD_SETTLE_S = 0.4              # let the board process frames before reading the diag block
FRAME_PERIOD_S = 0.02
DISARMED_FRAMES_BEFORE_ARM = 2


class ReadinessStreamer(bench.hw.FrameStreamer):
    """FrameStreamer that also records every HEARTBEAT system_status and STATUSTEXT with its time."""

    def __init__(self, file_descriptor):
        super().__init__(file_descriptor)
        self.heartbeats = []     # (time, system_status)
        self.texts = []          # (time, text)

    def run(self):
        """Same loop as the base class, with timestamped HEARTBEAT status and STATUSTEXT recording."""
        mav = bench.hw.mav
        serial_io = bench.serial_io
        next_frame = time.monotonic()
        while self.running:
            now = time.monotonic()
            if now >= next_frame:
                if not self.paused:
                    frame = self.generator.aux_channels(self.armed, list(self.motors), list(self.servos))
                    serial_io.write_all(self.fd, frame)
                next_frame += FRAME_PERIOD_S
                if next_frame < now - 1.0:
                    next_frame = now
            for message_id, payload in self.parser.feed(serial_io.read_available(self.fd)):
                if message_id == mav.MSG_HEARTBEAT:
                    heartbeat = mav.decode_heartbeat(payload)
                    self.board_armed = heartbeat["armed"]
                    self.heartbeat_time = time.monotonic()
                    self.heartbeats.append((self.heartbeat_time, heartbeat["system_status"]))
                elif message_id == mav.MSG_STATUSTEXT:
                    text = mav.decode_statustext(payload)["text"]
                    self.texts.append((time.monotonic(), text))
            time.sleep(0.001)

    def clear_records(self):
        """Forgets everything received so far (call after a board reset)."""
        self.heartbeats.clear()
        self.texts.clear()
        self.board_armed = None
        self.heartbeat_time = 0.0


def reset_satellite():
    """Resets the satellite over SWD (openocd: init; reset run; shutdown)."""
    command = ["openocd"] + bench.OPENOCD_ARGS + ["-c", "init; reset run; shutdown"]
    subprocess.run(command, capture_output=True, text=True, timeout=30)


def open_silent_streamer():
    """FTDI streamer that starts paused (sends nothing, listens only)."""
    file_descriptor = bench.serial_io.open_port(bench.FTDI_PORT, bench.MASTER_BAUD)
    streamer = ReadinessStreamer(file_descriptor)
    streamer.paused = True
    streamer.start()
    return streamer


def send_single_frame(streamer, armed):
    """Sends one AUX frame by hand (the streamer must be paused)."""
    frame = streamer.generator.aux_channels(armed, list(streamer.motors), list(streamer.servos))
    bench.serial_io.write_all(streamer.fd, frame)


def intervals(times):
    """Differences between consecutive times."""
    return [later - earlier for earlier, later in zip(times, times[1:])]


def phase_silent_link(report, streamer):
    """Phase 1: reset, no frames: UNINIT and NOT READY text at ~1 Hz."""
    report.start_phase("1", "reset, FTDI silent: UNINIT + NOT READY text")
    reset_satellite()
    streamer.clear_records()
    time.sleep(SILENT_OBSERVE_S)

    statuses = [status for _time, status in streamer.heartbeats]
    report.check("HEARTBEATs received", len(statuses) >= 2, "count=%d" % len(statuses))
    report.check("all HEARTBEAT system_status UNINIT", statuses and all(s == MAV_STATE_UNINIT for s in statuses),
                 "statuses=%s" % sorted(set(statuses)))

    texts = [text for _time, text in streamer.texts]
    text_times = [time_ for time_, _text in streamer.texts]
    report.check("NOT READY texts received (>= 2)", len(texts) >= 2, "count=%d" % len(texts))
    report.check("texts are NOT READY: LINK or BOOT_GRACE", texts and all(t in ACCEPTED_NOT_READY_TEXTS for t in texts),
                 "texts=%s" % sorted(set(texts)))

    text_intervals = intervals(text_times)
    in_band = text_intervals and all(HEARTBEAT_PERIOD_MIN_S <= gap <= HEARTBEAT_PERIOD_MAX_S for gap in text_intervals)
    report.check("NOT READY text period ~1 Hz", in_band,
                 "gaps_s=%s" % [round(gap, 2) for gap in text_intervals])
    if text_intervals:
        report.number("not_ready_text_period_mean", sum(text_intervals) / len(text_intervals), "s")


def phase_standby(report, streamer):
    """Phase 2: disarmed frames: STANDBY within ~1.5 s, NOT READY texts stop."""
    report.start_phase("2", "disarmed AUX frames: STANDBY, NOT READY texts stop")
    streamer.armed = False
    streamer.clear_records()
    start = time.monotonic()
    streamer.paused = False

    def standby_heartbeat_time():
        for heartbeat_time, status in list(streamer.heartbeats):
            if status == MAV_STATE_STANDBY:
                return heartbeat_time
        return None

    standby_time = bench.wait_until(standby_heartbeat_time, STANDBY_MAX_S + 1.0)
    standby_delay = None if standby_time is None else standby_time - start
    report.check("STANDBY within %.1f s of the first disarmed frame" % STANDBY_MAX_S,
                 standby_delay is not None and standby_delay <= STANDBY_MAX_S,
                 "delay=%s s" % (None if standby_delay is None else round(standby_delay, 2)))
    if standby_delay is not None:
        report.number("standby_delay", standby_delay, "s")
    if standby_time is None:
        return

    time.sleep(NO_TEXT_OBSERVE_S)
    late_texts = [text for time_, text in streamer.texts
                  if time_ > standby_time + TEXT_GRACE_S and text.startswith(NOT_READY_PREFIX)]
    report.check("no NOT READY text after STANDBY", not late_texts,
                 "texts=%s" % late_texts if late_texts else "observed %.1f s" % NO_TEXT_OBSERVE_S)

    later_statuses = [status for time_, status in streamer.heartbeats if time_ > standby_time]
    report.check("HEARTBEAT stays STANDBY", later_statuses and all(s == MAV_STATE_STANDBY for s in later_statuses),
                 "statuses=%s" % sorted(set(later_statuses)))
    report.check("HEARTBEAT disarmed", streamer.board_armed is False)


def phase_arm_while_not_ready(report, streamer):
    """Phase 3: reset, 2 disarmed frames, ARMED: arms despite not ready (edge counted), then disarm."""
    report.start_phase("3", "reset, 2 disarmed frames, ARMED edge while not ready: arms, then disarm")
    streamer.paused = True
    streamer.armed = False
    reset_satellite()
    time.sleep(BOOT_WAIT_S)
    streamer.clear_records()
    before = bench.diag()

    for _ in range(DISARMED_FRAMES_BEFORE_ARM):
        send_single_frame(streamer, armed=False)
        time.sleep(FRAME_PERIOD_S)
    streamer.armed = True
    send_single_frame(streamer, armed=True)
    arm_sent_at = time.monotonic()
    streamer.paused = False          # keep streaming ARMED frames

    armed_after = streamer.wait_board_state(True, ARM_REPORT_MAX_S, since=arm_sent_at)
    report.check("board arms (HEARTBEAT armed)", armed_after is not None,
                 "after=%s s" % (None if armed_after is None else round(armed_after, 2)))
    time.sleep(SWD_SETTLE_S)
    armed_diag = bench.diag()
    report.check("state ARMED over SWD", armed_diag["lastToState"] == bench.STATE_ARMED,
                 "lastToState=%d" % armed_diag["lastToState"])
    report.check("armEdges +1", armed_diag["armEdges"] - before["armEdges"] == 1,
                 "before=%d after=%d" % (before["armEdges"], armed_diag["armEdges"]))
    report.check("edgesNotReady +1", armed_diag["edgesNotReady"] - before["edgesNotReady"] == 1,
                 "before=%d after=%d" % (before["edgesNotReady"], armed_diag["edgesNotReady"]))
    report.check("lastNotReadyReason LINK (0)", armed_diag["lastNotReadyReason"] == REASON_LINK,
                 "value=%d" % armed_diag["lastNotReadyReason"])
    report.check("lastCause ARMED_FRAME (2)", armed_diag["lastCause"] == CAUSE_ARMED_FRAME,
                 "lastCause=%d" % armed_diag["lastCause"])

    streamer.armed = False
    disarm_sent_at = time.monotonic()
    disarmed_after = streamer.wait_board_state(False, DISARM_REPORT_MAX_S, since=disarm_sent_at)
    report.check("board disarms (HEARTBEAT disarmed)", disarmed_after is not None,
                 "after=%s s" % (None if disarmed_after is None else round(disarmed_after, 2)))
    time.sleep(SWD_SETTLE_S)
    disarmed_diag = bench.diag()
    report.check("state DISARMED over SWD", disarmed_diag["lastToState"] == bench.STATE_DISARMED,
                 "lastToState=%d" % disarmed_diag["lastToState"])
    report.check("lastCause DISARM_FRAME (3)", disarmed_diag["lastCause"] == CAUSE_DISARM_FRAME,
                 "lastCause=%d" % disarmed_diag["lastCause"])


def main():
    """Runs the three phases and writes summary.json."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    arguments = parser.parse_args()

    report = report_module.Report("test_hw_02_readiness", arguments.run_dir)
    ok, problems = bench.preflight()
    if not ok:
        report.setup_failed = True
        for problem in problems:
            report.check("preflight: " + problem, False)
        return report.finish()

    streamer = open_silent_streamer()
    try:
        phase_silent_link(report, streamer)
        phase_standby(report, streamer)
        phase_arm_while_not_ready(report, streamer)
    finally:
        streamer.armed = False           # always leave the board disarmed
        time.sleep(0.2)
        streamer.stop()
    return report.finish()


if __name__ == "__main__":
    sys.exit(main())
