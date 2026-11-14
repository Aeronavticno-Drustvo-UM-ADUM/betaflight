#!/usr/bin/env python3
"""HW 01: satellite servo outputs and disarmed motors (standalone bench, does not arm).

Phase 1: several commanded servo patterns, each servo pulse within tolerance, stable index -> pin mapping.
Phase 2: motor values in disarmed frames are ignored, motors stay at min_command (standard PWM, 1000 us).
Phase 3: pin level: timer channel output enabled (CCER, TIM1 MOE) and GPIO alternate-function push-pull for every pin.

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
    ("ascending", [1100, 1250, 1400, 1550, 1700, 1900]),
    ("descending", [1900, 1700, 1550, 1400, 1250, 1100]),
    ("centre", [1500] * hw.SERVO_COUNT),
    ("low-high alternating", [1000, 2000, 1000, 2000, 1000, 2000]),
    ("rotated", [1250, 1400, 1550, 1700, 1900, 1100]),
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
        elif len(set(commanded_us)) == len(commanded_us):     # repeated values make the mapping ambiguous
            results.check("servos '%s': same index -> pin mapping" % pattern_name, mapping == first_mapping,
                          "mapping %s" % {index + 1: pin for index, pin in mapping.items()})
    streamer.servos[:hw.SERVO_COUNT] = [1500] * hw.SERVO_COUNT


def phase_disarmed_motors(results, streamer):
    """Phase 2: motor values in disarmed frames are ignored, motors stay at min_command (1000 us, standard PWM)."""
    results.start_phase("2", "motors, disarmed")
    streamer.armed = False
    streamer.motors = list(DISARMED_MOTOR_VALUES)
    pulses = read_motor_pulses()
    idle_us = hw.MOTOR_MIN_US
    for pin in hw.MOTOR_PINS:
        error_us = abs(pulses[pin] - idle_us)
        results.check("motor %s stays at min_command (%.1f us)" % (pin, idle_us), error_us <= PULSE_TOLERANCE_US,
                      "measured %.1f us with commanded %s" % (pulses[pin], DISARMED_MOTOR_VALUES))
    streamer.motors = [hw.mav.PULSE_IGNORE] * hw.MOTOR_COUNT


def phase_pin_config(results):
    """Phase 3: every servo and motor pin has its timer output enabled and GPIO in AF push-pull mode (read over SWD)."""
    results.start_phase("3", "pin config, disarmed")
    config, afio_mapr = sat_bench.pin_config()
    for pin, pin_config in config.items():
        pin_ok = (pin_config["ccer_enabled"]
                  and pin_config["af_push_pull"]
                  and pin_config["moe"] is not False)      # moe is None for timers other than TIM1
        results.check("pin_%s_output" % pin, pin_ok,
                      "%s CH%d, CCER enabled %s, MOE %s, GPIO nibble 0x%X, AF push-pull %s" % (
                          pin_config["timer"], pin_config["channel"], pin_config["ccer_enabled"],
                          pin_config["moe"], pin_config["gpio_nibble"], pin_config["af_push_pull"]))
    results.number("AFIO_MAPR (0x%08X)" % afio_mapr, afio_mapr, "raw")       # info only, never a failure


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
        phase_pin_config(results)
    finally:
        if streamer is not None:
            streamer.stop()            # always stop the 50 Hz stream
    return results.finish()


if __name__ == "__main__":
    sys.exit(main())
