#!/usr/bin/env python3
"""Satellite hardware test 03: arm, armed motors, disarm, link-loss failsafe with exact delay, recovery.

Arms the board and spins the motors: remove the propellers (runner needs --props-off).
Run as: python3 test_hw_03_arm_failsafe.py --run-dir <dir>
"""
import argparse
import os
import sys
import time

HW_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HW_DIR)
from common import report, sat_bench  # noqa: E402

ARMS = True

TEST_NAME = "test_hw_03_arm_failsafe"
DISARMED_FRAMES_S = 1.0          # disarmed frames first, so the board sees an arm edge
SETTLE_S = 0.4                   # time for the board to apply a new command before SWD reads
HEARTBEAT_ARMED_MAX_S = 1.5      # HEARTBEAT is 1 Hz
STATE_WAIT_S = 3.0
LINK_TIMEOUT_MS = 500            # setting sd_link_timeout_ms default
DELAY_TOLERANCE_MS = 10.0
LINK_LOSS_REPEATS = 3
PULSE_TOLERANCE_US = 2.0
MIN_COMMAND = 1000
IDLE_PULSE_US = MIN_COMMAND      # standard PWM, 1000 us
ARMED_COMMAND = [1100, 1200, 1300]
CAUSE_LINK_TIMEOUT = 4
CAUSE_FAILSAFE_RECOVER = 5
MOTOR_COUNT = 3


def board_state():
    """Current satellite state (0 disarmed, 1 armed, 2 failsafe) from the SWD diag block."""
    return sat_bench.diag()["lastToState"]


def state_is(wanted_state):
    """Returns a condition function that is true while the board is in wanted_state."""
    def condition():
        return board_state() == wanted_state
    return condition


def worst_pulse_error_us(wanted_us_list):
    """Worst difference between sorted wanted and sorted measured motor pulses (pin order is not assumed)."""
    measured = sorted(sat_bench.motor_pulses_us().values())
    wanted = sorted(wanted_us_list)
    return max(abs(measured_us - wanted_us) for measured_us, wanted_us in zip(measured, wanted)), measured


def check_motors_idle(result, name):
    """Checks all motor pulses are at the idle width (1000 us)."""
    worst, measured = worst_pulse_error_us([IDLE_PULSE_US] * MOTOR_COUNT)
    result.check(name, worst <= PULSE_TOLERANCE_US,
                 "worst error %.1f us, measured %s" % (worst, [round(us, 1) for us in measured]))


def check_motors_armed(result, name):
    """Checks the motor pulses equal the commanded values."""
    wanted_us = list(ARMED_COMMAND)
    worst, measured = worst_pulse_error_us(wanted_us)
    result.check(name, worst <= PULSE_TOLERANCE_US,
                 "worst error %.1f us, measured %s" % (worst, [round(us, 1) for us in measured]))


def send_disarmed(streamer):
    """Master sends disarmed frames with motors at min_command."""
    streamer.armed = False
    streamer.motors = [MIN_COMMAND] * MOTOR_COUNT


def send_armed(streamer):
    """Master sends armed frames with the test motor commands."""
    streamer.armed = True
    streamer.motors = list(ARMED_COMMAND)


def arm_board(streamer, result, label):
    """Disarmed frames, then an arm edge; checks SWD state and HEARTBEAT. Returns True when ARMED."""
    send_disarmed(streamer)
    time.sleep(DISARMED_FRAMES_S)
    arm_time = time.monotonic()
    send_armed(streamer)
    armed_state = sat_bench.wait_until(state_is(sat_bench.STATE_ARMED), STATE_WAIT_S)
    result.check("%s: board ARMED (SWD state 1)" % label, armed_state)
    heartbeat_wait = streamer.wait_board_state(True, HEARTBEAT_ARMED_MAX_S + 5.0, since=arm_time)
    result.check("%s: HEARTBEAT armed within %.1f s" % (label, HEARTBEAT_ARMED_MAX_S),
                 heartbeat_wait is not None and heartbeat_wait <= HEARTBEAT_ARMED_MAX_S,
                 "after %s" % ("never" if heartbeat_wait is None else "%.2f s" % heartbeat_wait))
    time.sleep(SETTLE_S)
    return bool(armed_state)


def phase_arm_and_disarm(streamer, result):
    """Arm, check armed motors, disarm, check state 0 and idle motors. Returns True when the board armed."""
    result.start_phase("A", "arm, armed motors, disarm")
    armed = arm_board(streamer, result, "arm")
    if not armed:
        print("    board STATUSTEXT: %s" % (streamer.status_texts[-3:] or "none"))
        return False
    check_motors_armed(result, "armed motor pulses = commanded")
    disarm_time = time.monotonic()
    send_disarmed(streamer)
    disarmed_state = sat_bench.wait_until(state_is(sat_bench.STATE_DISARMED), STATE_WAIT_S)
    result.check("board DISARMED after disarmed frame (SWD state 0)", disarmed_state)
    result.check("HEARTBEAT reports disarmed",
                 streamer.wait_board_state(False, STATE_WAIT_S, since=disarm_time) is not None)
    time.sleep(SETTLE_S)
    check_motors_idle(result, "motors at 1000 us after disarm")
    return True


def measure_link_loss(streamer, result, index):
    """One link loss while armed: failsafe state, cause, delay, idle motors, then recovery."""
    streamer.paused = True                                  # master goes silent
    failsafe_state = sat_bench.wait_until(state_is(sat_bench.STATE_FAILSAFE), STATE_WAIT_S)
    result.check("loss %d: board FAILSAFE (SWD state 2)" % index, failsafe_state)
    if not failsafe_state:
        streamer.paused = False
        return
    fields = sat_bench.diag()
    result.check("loss %d: cause LINK_TIMEOUT (%d)" % (index, CAUSE_LINK_TIMEOUT),
                 fields["lastCause"] == CAUSE_LINK_TIMEOUT, "cause %d" % fields["lastCause"])
    delay_ms = sat_bench.failsafe_delay_ms()                # lastAuxUs stays frozen during the silence
    if delay_ms is None:
        result.check("loss %d: failsafe delay measurable" % index, False)
    else:
        result.number("failsafe_delay_ms_%d" % index, delay_ms, "ms")
        result.check("loss %d: failsafe delay %d +- %d ms" % (index, LINK_TIMEOUT_MS, DELAY_TOLERANCE_MS),
                     abs(delay_ms - LINK_TIMEOUT_MS) <= DELAY_TOLERANCE_MS, "%.1f ms" % delay_ms)
    check_motors_idle(result, "loss %d: motors at 1000 us in failsafe" % index)

    streamer.paused = False                                 # link returns with armed frames
    recovered_state = sat_bench.wait_until(state_is(sat_bench.STATE_ARMED), STATE_WAIT_S)
    result.check("loss %d: board ARMED again after the link returns" % index, recovered_state)
    cause = sat_bench.diag()["lastCause"]
    result.check("loss %d: cause FAILSAFE_RECOVER (%d)" % (index, CAUSE_FAILSAFE_RECOVER),
                 cause == CAUSE_FAILSAFE_RECOVER, "cause %d" % cause)
    time.sleep(SETTLE_S)
    check_motors_armed(result, "loss %d: armed motor pulses after recovery" % index)


def phase_link_loss(streamer, result):
    """Re-arm, then repeat the link loss LINK_LOSS_REPEATS times."""
    result.start_phase("B", "link-loss failsafe and recovery")
    if not arm_board(streamer, result, "re-arm"):
        return
    for index in range(1, LINK_LOSS_REPEATS + 1):
        measure_link_loss(streamer, result, index)
        if board_state() != sat_bench.STATE_ARMED:
            break


def phase_final_disarm(streamer, result):
    """Final disarm: state 0 and idle motors."""
    result.start_phase("C", "final disarm")
    send_disarmed(streamer)
    disarmed_state = sat_bench.wait_until(state_is(sat_bench.STATE_DISARMED), STATE_WAIT_S)
    result.check("final: board DISARMED (SWD state 0)", disarmed_state)
    time.sleep(SETTLE_S)
    check_motors_idle(result, "final: motors at 1000 us")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()
    result = report.Report(TEST_NAME, args.run_dir)
    streamer = None
    try:
        streamer = sat_bench.start_master(armed=False, motors=[MIN_COMMAND] * MOTOR_COUNT)
        if streamer.wait_board_state(False, 5.0) is None:
            result.setup_failed = True
            result.check("board answers with a disarmed HEARTBEAT", False)
        elif phase_arm_and_disarm(streamer, result):
            phase_link_loss(streamer, result)
    except Exception as error:                              # setup or tool failure: report it, still disarm
        result.check("test ran without exception", False, repr(error))
    finally:
        if streamer is not None:
            try:
                streamer.paused = False
                send_disarmed(streamer)
                time.sleep(0.3)
                phase_final_disarm(streamer, result)
            except Exception as error:
                result.check("final disarm without exception", False, repr(error))
            streamer.stop()                                 # always stop the streamer
    return result.finish()


if __name__ == "__main__":
    sys.exit(main())
