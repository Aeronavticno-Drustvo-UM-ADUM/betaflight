#!/usr/bin/env python3
"""Live view of the SATELLITEBOARD over SWD (ST-Link + OpenOCD), without halting the core.

Prints, every period: the values of the last ADUM_AUX_CHANNELS frame as the board applied them (servo outputs from
the timer registers, armed motor command motorPulse[] and the motor outputs), the satellite state and the link /
arming counters of servoDriverDiag. The frame itself is not kept in RAM, so these are the values it produced.

Run from the Betaflight repo root:  python3 test/tools/sat_monitor.py [--period 0.5]
Needs the firmware ELF that is on the board (obj/main/betaflight_SATELLITEBOARD.elf); one OpenOCD at a time.
"""

import argparse
import os
import socket
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_diag import FIELDS as DIAG_FIELDS  # noqa: E402

TCL_PORT = 6666
TIMER_CLOCK_HZ = 72000000                       # F103: TIM1 (APB2) and TIM2/TIM3 (APB1 x2) both run at 72 MHz
TIMER_BASE = {"TIM1": 0x40012C00, "TIM2": 0x40000000, "TIM3": 0x40000400}
REG_PSC = 0x28
REG_CCR1 = 0x34

# Function (frame index), PCB output header, timer, channel: target.c and docs/adum/sat_pinout.png (2026-11-14)
OUTPUTS = [
    ("S1", "O1", "TIM3", 1), ("S2", "O9", "TIM3", 4), ("S3", "O2", "TIM3", 2),
    ("S4", "O8", "TIM2", 2), ("S5", "O3", "TIM3", 3), ("S6", "O7", "TIM2", 1),
    ("M1", "O4", "TIM1", 1), ("M2", "O6", "TIM1", 2), ("M3", "O5", "TIM1", 3),
]
STATE_NAMES = {0: "DISARMED", 1: "ARMED", 2: "FAILSAFE"}
NOT_READY_NAMES = {0: "LINK", 1: "BOOT_GRACE", 2: "MOTOR_CONFIG", 3: "CONFIG_ACTIVE", 4: "READY"}
CAUSE_NAMES = {0: "NONE", 1: "FIRST_FRAME_ARMED", 2: "ARMED_FRAME", 3: "DISARM_FRAME", 4: "LINK_TIMEOUT",
               5: "FAILSAFE_RECOVER"}


def find_symbols(elf_path):
    """Addresses of motorPulse, the satellite state and servoDriverDiag from nm (state is an LTO-renamed static)."""
    output = subprocess.check_output(["nm", elf_path], text=True)
    addresses = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        name = parts[2]
        if name in ("motorPulse", "servoDriverDiag"):
            addresses[name] = int(parts[0], 16)
        elif name == "state" or name.startswith("state.lto_priv"):
            addresses["state"] = int(parts[0], 16)
    missing = {"motorPulse", "servoDriverDiag", "state"} - set(addresses)
    if missing:
        sys.exit("symbols not found in %s: %s" % (elf_path, sorted(missing)))
    return addresses


class OpenOcd:
    """One OpenOCD process with its Tcl RPC socket; memory reads never halt the core."""

    def __init__(self, openocd_args):
        self.log = tempfile.TemporaryFile()                 # OpenOCD output is not read: a full pipe would stall it
        arguments = ["openocd"] + openocd_args.split() + [
            "-c", "tcl_port %d" % TCL_PORT, "-c", "telnet_port disabled", "-c", "gdb_port disabled", "-c", "init"]
        self.process = subprocess.Popen(arguments, stdout=self.log, stderr=self.log)
        self.sock = self._connect()

    def _connect(self):
        """Connects to the Tcl port, retrying while OpenOCD starts."""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                return socket.create_connection(("127.0.0.1", TCL_PORT), timeout=2)
            except OSError:
                time.sleep(0.2)
        self.close()
        sys.exit("OpenOCD did not start (ST-Link connected? another OpenOCD running?)")

    def words(self, address, count):
        """Reads count 32-bit words starting at address."""
        self.sock.sendall(("read_memory 0x%08x 32 %d" % (address, count)).encode() + b"\x1a")
        reply = b""
        while not reply.endswith(b"\x1a"):
            reply += self.sock.recv(4096)
        return [int(token, 16) for token in reply[:-1].decode().split()]

    def close(self):
        """Stops OpenOCD."""
        self.process.terminate()
        self.process.wait(timeout=5)


def output_pulses_us(openocd):
    """Pulse width in us per output, from each timer's prescaler and compare registers."""
    timer_registers = {}
    for timer, base in TIMER_BASE.items():
        timer_registers[timer] = openocd.words(base + REG_PSC, 7)       # PSC .. CCR4 in one read
    pulses = {}
    for name, _output, timer, channel in OUTPUTS:
        registers = timer_registers[timer]
        prescaler = registers[0] + 1
        compare = registers[2 + channel]                                # channel is 1-based; CCR1 is word 3 after PSC
        pulses[name] = compare * prescaler * 1000000.0 / TIMER_CLOCK_HZ
    return pulses


def read_snapshot(openocd, addresses):
    """One snapshot: output pulses, motorPulse[], state byte and the diag fields."""
    motor_words = openocd.words(addresses["motorPulse"], 2)
    motor_command = [motor_words[0] & 0xFFFF, motor_words[0] >> 16, motor_words[1] & 0xFFFF]
    state_word = openocd.words(addresses["state"] & ~3, 1)[0]
    state = (state_word >> (8 * (addresses["state"] & 3))) & 0xFF
    diag = dict(zip(DIAG_FIELDS, openocd.words(addresses["servoDriverDiag"], len(DIAG_FIELDS))))
    return output_pulses_us(openocd), motor_command, state, diag


def print_snapshot(snapshot, previous_diag, elapsed_s):
    """Prints one block: outputs, arming info, link counters with per-second rates."""
    pulses, motor_command, state, diag = snapshot
    frame_rate = 0.0
    armed_frames = 0
    if previous_diag is not None and elapsed_s > 0:
        frame_rate = (diag["auxFrames"] - previous_diag["auxFrames"]) / elapsed_s
        armed_frames = diag["auxArmedFrames"] - previous_diag["auxArmedFrames"]
    received_arming = "ARMED" if armed_frames > 0 else "DISARMED"

    print("-" * 78)
    print("state %-8s  frames %5.1f/s  last frames say %-8s  boot %d  parse errors %d  tx drops %d" % (
        STATE_NAMES.get(state, state), frame_rate, received_arming, diag["bootCount"], diag["parseErrors"],
        diag["txDrops"]))
    print("servos  " + "  ".join("%s(%s) %6.1f" % (name, output, pulses[name])
                                 for name, output, _t, _c in OUTPUTS if name.startswith("S")))
    print("motors  " + "  ".join("%s(%s) out %6.1f cmd %4d" % (name, output, pulses[name], motor_command[index])
                                 for index, (name, output, _t, _c) in enumerate(OUTPUTS[6:])))
    print("last transition %s -> %s (%s) at frame %d   arm edges %d (not ready %d)   last not-ready %s" % (
        STATE_NAMES.get(diag["lastFromState"], diag["lastFromState"]),
        STATE_NAMES.get(diag["lastToState"], diag["lastToState"]),
        CAUSE_NAMES.get(diag["lastCause"], diag["lastCause"]), diag["lastAuxFrameNo"], diag["armEdges"],
        diag["edgesNotReady"], NOT_READY_NAMES.get(diag["lastNotReadyReason"], diag["lastNotReadyReason"])))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", default="obj/main/betaflight_SATELLITEBOARD.elf")
    parser.add_argument("--openocd-args", default="-f interface/stlink.cfg -f target/stm32f1x.cfg")
    parser.add_argument("--period", type=float, default=0.5, help="seconds between prints")
    args = parser.parse_args()

    addresses = find_symbols(args.elf)
    openocd = OpenOcd(args.openocd_args)
    previous_diag = None
    previous_time = time.monotonic()
    try:
        while True:
            snapshot = read_snapshot(openocd, addresses)
            now = time.monotonic()
            print_snapshot(snapshot, previous_diag, now - previous_time)
            previous_diag = snapshot[3]
            previous_time = now
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        openocd.close()


if __name__ == "__main__":
    main()
