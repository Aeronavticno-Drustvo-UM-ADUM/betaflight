#!/usr/bin/env python3
"""HW 01: satellite servo outputs and disarmed motors (standalone bench, does not arm).

Phase 1: several commanded servo patterns, each servo pulse within tolerance, stable index -> pin mapping.
Phase 2: motor values in disarmed frames are ignored, motors stay at min_command (OneShot125: 125 us).

Run via test/hw/run_hw_tests.py, or: python3 test_hw_01_outputs.py --run-dir DIR
Exit codes: 0 passed, 1 check failed, 2 setup impossible.
"""
import argparse
import os
import sys

HW_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HW_DIR)
from common import report, sat_bench  # noqa: E402

hw = sat_bench.hw                      # test/tools/hw_output_check (pins, constants, match_outputs)
PULSE_TOLERANCE_US = 2.0               # same default as hw_output_check --tol-us
SETTLE_S = 0.4                         # board processes a few frames before the pulses are read
SERVO_PATTERNS = [
    ("ascending", [1100, 1300, 1500, 1700, 1900]),
    ("descending", [1900, 1700, 1500, 1300, 1100]),
    ("centre", [1500] * hw.SERVO_COUNT),
    ("low-high alternating", [1000, 2000, 1000, 2000, 1000]),
    ("rotated", [1300, 1500, 1700, 1900, 1100]),
]
DISARMED_MOTOR_VALUES = [1300, 1400, 1500]      # must be ignored while the frames are disarmed


def read_servo_pulses():
    """Waits for the board to settle, then reads the servo pulse widths in us {pin: us} over SWD."""
    sat_bench.time.sleep(SETTLE_S)
    return sat_bench.servo_pulses_us()


def read_motor_pulses():
    """Waits for the board to settle, then reads the motor pulse widths in us {pin: us} over SWD."""
    sat_bench.time.sleep(SETTLE_S)
    return sat_bench.motor_pulses_us()


def phase_servos(results, streamer):
    """Phase 1: each commanded servo pattern shows up on the pins within tolerance, mapping stays the same."""
    results.start_phase("1", "servos, disarmed")
    streamer.armed = False
    first_mapping = None
    for pattern_name, commanded_us in SERVO_PATTERNS:
        streamer.servos[:hw.SERVO_COUNT] = commanded_us
        pulses = read_servo_pulses()
        mapping, worst_error_us = hw.match_outputs(commanded_us, hw.SERVO_PINS, pulses, PULSE_TOLERANCE_US)
        measured = {pin: round(pulses[pin], 1) for pin in hw.SERVO_PINS}
        results.check("servos '%s': widths within %.0f us" % (pattern_name, PULSE_TOLERANCE_US),
                      worst_error_us <= PULSE_TOLERANCE_US,
                      "worst error %.1f us, commanded %s, measured %s" % (worst_error_us, commanded_us, measured))
        results.number("servo worst error '%s'" % pattern_name, worst_error_us, "us")
        if first_mapping is None:
            first_mapping = mapping
            print("    servo index -> pin: %s" % {index + 1: pin for index, pin in mapping.items()})
        else:
            results.check("servos '%s': same index -> pin mapping" % pattern_name, mapping == first_mapping,
                          "mapping %s" % {index + 1: pin for index, pin in mapping.items()})
    streamer.servos[:hw.SERVO_COUNT] = [1500] * hw.SERVO_COUNT


def phase_disarmed_motors(results, streamer):
    """Phase 2: motor values in disarmed frames are ignored, motors stay at min_command / 8 (125 us)."""
    results.start_phase("2", "motors, disarmed")
    streamer.armed = False
    streamer.motors = list(DISARMED_MOTOR_VALUES)
    pulses = read_motor_pulses()
    idle_us = hw.MOTOR_MIN_US / hw.ONESHOT125_DIVIDER
    for pin in hw.MOTOR_PINS:
        error_us = abs(pulses[pin] - idle_us)
        results.check("motor %s stays at min_command (%.1f us)" % (pin, idle_us), error_us <= PULSE_TOLERANCE_US,
                      "measured %.1f us with commanded %s" % (pulses[pin], DISARMED_MOTOR_VALUES))
    streamer.motors = [hw.mav.PULSE_IGNORE] * hw.MOTOR_COUNT


def run_preflight(results):
    """Runs the bench preflight; marks the run as setup failure (exit 2) and returns False when it fails."""
    try:
        preflight_ok, problems = sat_bench.preflight()
    except Exception as error:         # missing device, OpenOCD failure: the setup is not usable
        preflight_ok, problems = False, ["preflight raised %s: %s" % (type(error).__name__, error)]
    for problem in problems:
        print("PREFLIGHT: " + problem, flush=True)
    if not preflight_ok:
        results.setup_failed = True
    return preflight_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", required=True, help="directory for summary.json")
    args = parser.parse_args()

    results = report.Report("test_hw_01_outputs", args.run_dir)
    results.start_phase("0", "preflight")
    if not run_preflight(results):
        return results.finish()

    streamer = None
    try:
        streamer = sat_bench.start_master(armed=False, servos=[1500] * hw.SERVO_COUNT)
        phase_servos(results, streamer)
        phase_disarmed_motors(results, streamer)
    finally:
        if streamer is not None:
            streamer.stop()            # always stop the 50 Hz stream
    return results.finish()


if __name__ == "__main__":
    sys.exit(main())
