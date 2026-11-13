"""Check reporting in the style of the SITL tests ([PASS]/[FAIL]/[KNOWN]/[RISK], SUMMARY line) plus summary.json.

[RISK]: a measured hazard that is spec-conformant or outside any spec (what, number, why it matters); never fails
the run.

Exit codes: 0 all checks passed, 1 a check failed, 2 setup impossible (ports busy, device missing, wrong wiring).
"""
import datetime
import json
import os
import subprocess
import time

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_SETUP = 2


def git_info(repo):
    """Commit and dirty flag of a git work tree; None values when not available."""
    try:
        commit = subprocess.run(["git", "-C", repo, "rev-parse", "--short=9", "HEAD"], capture_output=True,
                                text=True, check=True).stdout.strip()
        status = subprocess.run(["git", "-C", repo, "status", "--porcelain", "--untracked-files=no"],
                                capture_output=True, text=True, check=True).stdout
        branch = subprocess.run(["git", "-C", repo, "branch", "--show-current"], capture_output=True,
                                text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None, "branch": None}
    return {"commit": commit, "branch": branch, "dirty": bool(status.strip())}


class Report:
    """Collects checks, known issues, measured numbers and phase timings of one test run."""

    def __init__(self, test_name, run_dir):
        self.test_name = test_name
        self.run_dir = run_dir
        self.checks = []              # {"phase", "name", "ok", "detail"}
        self.known = []
        self.risks = []               # {"phase", "name", "value", "unit", "why"}
        self.numbers = {}             # name -> {"value", "unit", "phase"}
        self.phases = []              # {"name", "start_s", "duration_s"}
        self.metadata = {}
        self.phase = "setup"
        self.setup_failed = False
        self.t_begin = time.monotonic()
        self.started_at = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
        self._phase_begin = self.t_begin

    def start_phase(self, name, title):
        """Closes the current phase and starts the next one."""
        self._close_phase()
        self.phase = name
        self._phase_begin = time.monotonic()
        print("%s. %s" % (name, title), flush=True)

    def _close_phase(self):
        now = time.monotonic()
        self.phases.append({"name": self.phase, "start_s": round(self._phase_begin - self.t_begin, 3),
                            "duration_s": round(now - self._phase_begin, 3)})

    def check(self, name, ok, detail=""):
        ok = bool(ok)
        self.checks.append({"phase": self.phase, "name": name, "ok": ok, "detail": detail})
        suffix = " - " + detail if detail else ""
        print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name, suffix), flush=True)
        return ok

    def known_issue(self, name, observed):
        """Documents current behaviour that is an open design question; never fails the run."""
        self.known.append({"phase": self.phase, "name": name, "observed": observed})
        print("  [KNOWN] %s - observed: %s" % (name, observed), flush=True)

    def risk(self, name, value, unit, why):
        """Documents a measured hazard (spec-conformant or outside any spec); never fails the run."""
        if isinstance(value, float):
            value = round(value, 4)
        self.risks.append({"phase": self.phase, "name": name, "value": value, "unit": unit, "why": why})
        print("  [RISK] %s: %s %s - %s" % (name, value, unit, why), flush=True)

    def number(self, name, value, unit):
        """Records a measured number (printed and stored in summary.json)."""
        if isinstance(value, float):
            value = round(value, 4)
        self.numbers[name] = {"value": value, "unit": unit, "phase": self.phase}
        print("  %-44s %s %s" % (name, value, unit), flush=True)

    def failed_count(self):
        return sum(1 for check in self.checks if not check["ok"])

    def exit_code(self):
        if self.setup_failed:
            return EXIT_SETUP
        return EXIT_FAILED if self.failed_count() else EXIT_OK

    def finish(self, extra=None):
        """Prints the SUMMARY line, writes summary.json, returns the exit code."""
        self._close_phase()
        code = self.exit_code()
        summary = {
            "test": self.test_name,
            "started_at": self.started_at,
            "duration_s": round(time.monotonic() - self.t_begin, 3),
            "exit_code": code,
            "result": {EXIT_OK: "PASS", EXIT_FAILED: "FAIL", EXIT_SETUP: "SETUP_ERROR"}[code],
            "checks_total": len(self.checks),
            "checks_failed": self.failed_count(),
            "known_issues": len(self.known),
            "risks_total": len(self.risks),
            "metadata": self.metadata,
            "phases": self.phases,
            "numbers": self.numbers,
            "checks": self.checks,
            "known": self.known,
            "risks": self.risks,
        }
        if extra:
            summary.update(extra)
        os.makedirs(self.run_dir, exist_ok=True)
        with open(os.path.join(self.run_dir, "summary.json"), "w") as out:
            json.dump(summary, out, indent=2)
        print("\nSUMMARY checks=%d failed=%d known=%d risks=%d time=%.1fs run_dir=%s"
              % (len(self.checks), self.failed_count(), len(self.known), len(self.risks), summary["duration_s"],
                 self.run_dir),
              flush=True)
        return code


def median(values):
    ordered = sorted(values)
    count = len(ordered)
    if count == 0:
        return None
    if count % 2:
        return ordered[count // 2]
    return 0.5 * (ordered[count // 2 - 1] + ordered[count // 2])


def percentile(values, fraction):
    """Nearest-rank percentile (fraction 0..1); None for no values."""
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]
