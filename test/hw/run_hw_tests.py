#!/usr/bin/env python3
"""Runner of the standalone satellite hardware tests (FTDI fake master + ST-Link SWD).

  test/hw/run_hw_tests.py [NAME ...] [--repeat N] [--props-off]

Tests are test/hw/test_hw_*.py, each run as `python3 <file> --run-dir <dir>`; NAME selects by substring.
Run dir: test/hw/runs/<YYYYmmdd_HHMMSS>_<test>_r<k>/ with output.log and summary.json.
Tests that arm declare ARMS = True at module level and need --props-off.
Exit code: 0 all passed, 1 a check or file failed, 2 setup not possible (device missing, wrong wiring).
"""
import argparse
import datetime
import glob
import json
import os
import re
import subprocess
import sys

HW_DIR = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(HW_DIR, "runs")
sys.path.insert(0, HW_DIR)
from common import report  # noqa: E402


def discover_tests(names):
    """Test files test_hw_*.py, filtered by NAME substrings (all when no name is given)."""
    files = sorted(glob.glob(os.path.join(HW_DIR, "test_hw_*.py")))
    if not names:
        return files
    return [path for path in files if any(name in os.path.basename(path) for name in names)]


def test_arms(path):
    """True when the test file declares ARMS = True at module level."""
    with open(path) as source:
        return re.search(r"^ARMS\s*=\s*True\b", source.read(), re.MULTILINE) is not None


def run_test_file(path, run_dir):
    """Runs one test file as a subprocess, output to console and run_dir/output.log. Returns its exit code."""
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "output.log")
    command = [sys.executable, path, "--run-dir", run_dir]
    with open(log_path, "w") as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in process.stdout:
            log.write(line)
            if any(tag in line for tag in ("[FAIL]", "[RISK]", "SUMMARY")):
                print("    " + line.rstrip(), flush=True)
        return process.wait()


def read_summary(run_dir):
    """summary.json of a run, or None when the test did not write one."""
    try:
        with open(os.path.join(run_dir, "summary.json")) as summary_file:
            return json.load(summary_file)
    except (OSError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("names", nargs="*", help="test name substrings (default: all)")
    parser.add_argument("--repeat", type=int, default=1, help="runs per test file")
    parser.add_argument("--props-off", action="store_true", help="confirm propellers are off (needed for ARMS tests)")
    args = parser.parse_args()

    tests = discover_tests(args.names)
    if not tests:
        print("no test files test_hw_*.py match")
        return report.EXIT_SETUP

    arming_tests = [os.path.basename(path) for path in tests if test_arms(path)]
    if arming_tests and not args.props_off:
        print("these tests arm and spin the motors: %s\nremove the propellers and pass --props-off" % ", ".join(arming_tests))
        return report.EXIT_SETUP

    from common import sat_bench  # imported late: --help needs no tools
    preflight_ok, problems = sat_bench.preflight()
    if not preflight_ok:
        for problem in problems:
            print("PREFLIGHT: " + problem)
        return report.EXIT_SETUP

    worst_code = report.EXIT_OK
    for path in tests:
        test_name = os.path.splitext(os.path.basename(path))[0]
        for repeat_index in range(1, args.repeat + 1):
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            run_dir = os.path.join(RUNS_DIR, "%s_%s_r%d" % (stamp, test_name, repeat_index))
            exit_code = run_test_file(path, run_dir)
            summary = read_summary(run_dir)
            result = summary["result"] if summary else "NO_SUMMARY"
            print("%-28s r%d  %-11s exit=%d  %s" % (test_name, repeat_index, result, exit_code, run_dir), flush=True)
            if exit_code == report.EXIT_SETUP:
                worst_code = report.EXIT_SETUP
            elif exit_code != report.EXIT_OK and worst_code == report.EXIT_OK:
                worst_code = report.EXIT_FAILED
    return worst_code


if __name__ == "__main__":
    sys.exit(main())
