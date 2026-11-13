#!/usr/bin/env python3
"""Run the SITL test files test/test_*.py one after the other (they share the tcp ports 5761-5763).

  test/run_tests.py [--build] [--keep] [--verbose] [NAME ...]

--build rebuilds the SITL binary once, before the first file. NAME selects files by substring (e.g. 02 or link_loss).
The output of every file is written to test/sitl-run-logs/<file>.log; failing lines are printed here.
Exit code: 0 all checks passed, 1 a check or a file failed, 2 the SITL setup was not possible (e.g. ports busy).
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from common import harness  # noqa: E402

LOG_DIR = os.path.join(HERE, "sitl-run-logs")        # ignored by git (/test/sitl-run*/)
SUMMARY = re.compile(r"^SUMMARY checks=(\d+) failed=(\d+) known=(\d+)", re.M)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("names", nargs="*")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(HERE, "test_*.py")))
    if args.names:
        files = [f for f in files if any(n in os.path.basename(f) for n in args.names)]
    if args.build or not os.path.exists(harness.ELF):
        harness.build()
    os.makedirs(LOG_DIR, exist_ok=True)

    rows, worst = [], 0
    for path in files:
        name = os.path.basename(path)
        cmd = [sys.executable, path] + (["--keep"] if args.keep else []) + (["--verbose"] if args.verbose else [])
        begin = time.monotonic()
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        seconds = time.monotonic() - begin
        with open(os.path.join(LOG_DIR, name[:-3] + ".log"), "w") as log:
            log.write(proc.stdout)
        match = SUMMARY.search(proc.stdout)
        checks, failed, known = (int(g) for g in match.groups()) if match else (0, 0, 0)
        code = proc.returncode if match or proc.returncode else 1
        worst = 2 if (worst == 2 or code == 2) else max(worst, 1 if (failed or code) else 0)
        rows.append((name, checks, failed, known, seconds, code))
        for line in proc.stdout.splitlines():
            if "[FAIL]" in line or "[KNOWN]" in line or line.startswith(("failsafe delay", "parser recovery", "not-ready STATUSTEXT", "motor writes")):
                print("  %s: %s" % (name, line.strip()))
        if not match:
            print("  %s: no SUMMARY line, exit %d, see %s" % (name, proc.returncode, LOG_DIR))

    print("\n%-30s %7s %7s %7s %9s" % ("file", "checks", "failed", "known", "time [s]"))
    for name, checks, failed, known, seconds, code in rows:
        print("%-30s %7d %7d %7d %9.1f%s" % (name, checks, failed, known, seconds, "" if code == 0 else "  exit %d" % code))
    print("%-30s %7d %7d %7d %9.1f" % ("total", sum(r[1] for r in rows), sum(r[2] for r in rows),
                                         sum(r[3] for r in rows), sum(r[4] for r in rows)))
    return worst


if __name__ == "__main__":
    sys.exit(main())
