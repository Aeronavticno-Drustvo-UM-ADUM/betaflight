#!/usr/bin/env python3
"""Test the SATELLITEBOARD servo driver on SITL, no hardware needed.

  test/test_servo_sitl.py [--build] [--keep] [--verbose]

1. builds SITL with USE_SERVO_DRIVER into obj/main_servotest (separate from the stock SITL build)
2. applies test/servo_config.txt through the CLI on tcp://127.0.0.1:5761 and saves it
3. restarts SITL, connects to the MAVLink port (UART2, tcp://127.0.0.1:5762)
4. sends arm + RC_CHANNELS_OVERRIDE frames and checks the servo/motor outputs that SITL prints

Uses only the Python standard library. Exit code 0 when every check passes.
"""
import argparse
import os
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from mavlink_gen import MavlinkGenerator, CRC_EXTRA, MSG_HEARTBEAT, MAV_MODE_FLAG_SAFETY_ARMED, mavlink_crc  # noqa: E402

OBJ_DIR = os.path.join(ROOT, "obj", "main_servotest")
ELF = os.path.join(OBJ_DIR, "betaflight_SITL.elf")
RUN_DIR = os.path.join(HERE, "sitl-run-servotest")
CONFIG = os.path.join(HERE, "servo_config.txt")
CLI_PORT, MAVLINK_PORT = 5761, 5762

CENTRE, MOTOR_MIN, MOTOR_MAX = 1500, 1000, 2000
SERVOS, MOTORS = 5, 3


class Sitl:
    """Runs the SITL binary and tracks the [servo]/[motor] lines it prints."""

    def __init__(self, verbose):
        self.verbose = verbose
        self.proc = None
        self.servo = {}
        self.motor = {}
        self.history = []          # (time, kind, index, value)
        self.lines = []
        self.lock = threading.Lock()

    def start(self):
        os.makedirs(RUN_DIR, exist_ok=True)
        self.proc = subprocess.Popen(["stdbuf", "-o0", "-e0", ELF], cwd=RUN_DIR, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        for line in self.proc.stdout:
            line = line.rstrip()
            with self.lock:
                self.lines.append(line)
                for kind, table in (("[servo]", self.servo), ("[motor]", self.motor)):
                    if line.startswith(kind):
                        index, value = line[len(kind):].split("=")
                        table[int(index)] = int(value)
                        self.history.append((time.monotonic(), kind, int(index), int(value)))
            if self.verbose and not line.startswith("[FLASH_"):
                print("   sitl|", line)

    def wait_for(self, text, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if any(text in l for l in self.lines):
                    return True
            if self.proc.poll() is not None:
                return False
            time.sleep(0.05)
        return False

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def values(self, kind):
        with self.lock:
            return dict(self.servo if kind == "servo" else self.motor)

    def changes(self, kind, index, since):
        tag = "[%s]" % kind
        with self.lock:
            return [v for (t, k, i, v) in self.history if t >= since and k == tag and i == index]


def connect(port, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=2)
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("cannot connect to tcp port %d" % port)


def read_until(sock, marker, timeout=3.0):
    sock.settimeout(0.2)
    data, end = b"", time.monotonic() + timeout
    while time.monotonic() < end and marker not in data:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        data += chunk
    return data.decode(errors="replace")


def apply_config(verbose):
    """Send servo_config.txt through the CLI. The last line (save) restarts SITL."""
    commands = []
    with open(CONFIG) as f:
        for raw in f:
            line = raw.split("#")[0].strip() if raw.lstrip().startswith("#") else raw.strip()
            if line:
                commands.append(line)
    sock = connect(CLI_PORT)
    sock.sendall(b"#")
    banner = read_until(sock, b"# ", 3.0)
    if "CLI" not in banner:
        raise RuntimeError("CLI did not start: %r" % banner)
    problems = []
    for cmd in commands:
        sock.sendall(cmd.encode() + b"\n")
        if cmd == "save":
            break
        reply = read_until(sock, b"\n# ", 2.0)
        if verbose:
            print("   cli|", cmd, "->", reply.strip().replace("\r\n", " | "))
        if "Invalid" in reply or "Parse error" in reply or "ERROR" in reply:
            problems.append("%s -> %s" % (cmd, reply.strip()))
    sock.close()
    return problems


class BoardLink:
    """MAVLink connection to the board: sends frames and decodes the heartbeat it returns."""

    def __init__(self, sock):
        self.sock = sock
        self.gen = MavlinkGenerator()
        self.rx = bytearray()
        self.board_armed = None
        self.heartbeats = 0
        self.stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        self.sock.settimeout(0.2)
        while not self.stop:
            try:
                chunk = self.sock.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            self.rx += chunk
            self._parse()

    def _parse(self):
        while True:
            start = self.rx.find(b"\xfe")
            if start < 0:
                self.rx.clear()
                return
            del self.rx[:start]
            if len(self.rx) < 8:
                return
            length = self.rx[1]
            total = length + 8
            if len(self.rx) < total:
                return
            msgid = self.rx[5]
            crc = self.rx[total - 2] | (self.rx[total - 1] << 8)
            if msgid in CRC_EXTRA and mavlink_crc(bytes(self.rx[1:total - 2]), CRC_EXTRA[msgid]) == crc:
                if msgid == MSG_HEARTBEAT:
                    self.board_armed = bool(self.rx[6 + 6] & MAV_MODE_FLAG_SAFETY_ARMED)
                    self.heartbeats += 1
                del self.rx[:total]
            else:
                del self.rx[:1]

    def send(self, data):
        self.sock.sendall(data)

    def stream(self, seconds, armed_heartbeat, channels, rate_hz=20, step=None):
        """Send RC_CHANNELS_OVERRIDE (and optionally a HEARTBEAT) at rate_hz for `seconds`.
        step(fraction) may return a new channel list for sweeps."""
        end = time.monotonic() + seconds
        begin = time.monotonic()
        while time.monotonic() < end:
            if step:
                channels = step((time.monotonic() - begin) / seconds)
            if armed_heartbeat is not None:
                self.send(self.gen.heartbeat(armed_heartbeat))
            self.send(self.gen.rc_override(channels))
            time.sleep(1.0 / rate_hz)

    def close(self):
        self.stop = True
        self.sock.close()


results = []


def check(name, ok, detail=""):
    results.append(ok)
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name, (" - " + detail) if detail else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", action="store_true", help="rebuild the SITL test binary")
    ap.add_argument("--keep", action="store_true", help="keep the SITL config between runs")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    if args.build or not os.path.exists(ELF):
        print("Building SITL with USE_SERVO_DRIVER ...")
        subprocess.run(["make", "-C", ROOT, "TARGET=SITL", "OBJECT_DIR=" + OBJ_DIR, "EXTRA_FLAGS=-DUSE_SERVO_DRIVER",
                        "-j%d" % os.cpu_count()], check=True, stdout=subprocess.DEVNULL)

    if not args.keep and os.path.exists(os.path.join(RUN_DIR, "eeprom.bin")):
        os.remove(os.path.join(RUN_DIR, "eeprom.bin"))

    for port in (CLI_PORT, MAVLINK_PORT):
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.5).close()
        except OSError:
            continue
        print("tcp port %d is already in use: another SITL is running? (pkill -x betaflight_SITL)" % port)
        return 2

    sitl = Sitl(args.verbose)
    link = None
    try:
        print("1. Configure (servo_config.txt over the CLI)")
        sitl.start()
        if not sitl.wait_for("bind port %d" % CLI_PORT):
            check("SITL starts", False, "no CLI port; check %s" % ELF)
            return 1
        problems = apply_config(args.verbose)
        check("config accepted by the CLI", not problems, "; ".join(problems))
        time.sleep(1.0)
        sitl.stop()

        print("2. Restart with the saved config")
        sitl = Sitl(args.verbose)
        sitl.start()
        check("MAVLink port is open on UART2", sitl.wait_for("bind port %d" % MAVLINK_PORT),
              "serial port with the MAVLink function was not opened")
        time.sleep(0.5)
        link = BoardLink(connect(MAVLINK_PORT))

        print("3. Outputs after start-up (safe state)")
        check("servos centred", all(sitl.values("servo").get(i) == CENTRE for i in range(SERVOS)), str(sitl.values("servo")))
        check("motors at min_command", all(sitl.values("motor").get(i) == MOTOR_MIN for i in range(MOTORS)), str(sitl.values("motor")))
        time.sleep(1.2)
        check("board sends a HEARTBEAT", link.heartbeats > 0)

        print("4. Disarmed: RC override must not move anything")
        link.stream(1.0, False, [1800, 1200, 1700, 1300, 1600, 1400, 1400, 1400])
        check("servos stay centred", all(sitl.values("servo").get(i) == CENTRE for i in range(SERVOS)), str(sitl.values("servo")))
        check("motors stay at min_command", all(sitl.values("motor").get(i) == MOTOR_MIN for i in range(MOTORS)), str(sitl.values("motor")))

        print("5. Armed: outputs follow RC_CHANNELS_OVERRIDE")
        target = [1800, 1200, 1700, 1300, 1600, 1400, 1500, 1600]
        link.stream(1.0, True, target)
        servos, motors = sitl.values("servo"), sitl.values("motor")
        check("servos 0-4 follow channels 1-5", [servos.get(i) for i in range(5)] == target[:5], str(servos))
        check("motors 0-2 follow channels 6-8", [motors.get(i) for i in range(3)] == target[5:], str(motors))
        check("board reports armed in its HEARTBEAT", link.board_armed is True)

        print("6. Servo 0 sweep 1000 -> 2000 (are the servos moving?)")
        since = time.monotonic()
        link.stream(2.0, True, target, step=lambda f: [int(1000 + 1000 * f)] + target[1:])
        sweep = sitl.changes("servo", 0, since)
        check("servo 0 changes continuously", len(set(sweep)) >= 20, "%d distinct values" % len(set(sweep)))
        check("servo 0 sweep is monotonic", sweep == sorted(sweep), "first %s last %s" % (sweep[:1], sweep[-1:]))
        check("servo 0 reaches the end of the range", bool(sweep) and sweep[-1] >= 1950, "last %s" % sweep[-1:])

        print("7. Clamping and ignored channels")
        link.stream(0.5, True, [3000, 100, None, 0, 1500, 3000, None, 1100])
        servos, motors = sitl.values("servo"), sitl.values("motor")
        check("servo pulse clamped to 2500 / 500", servos.get(0) == 2500 and servos.get(1) == 500, str(servos))
        check("channels 0 / 65535 are ignored", servos.get(2) == target[2] and servos.get(3) == target[3], str(servos))
        check("motor clamped to max_throttle", motors.get(0) == MOTOR_MAX and motors.get(1) == target[6], str(motors))

        print("8. Link loss: RC stops, the board must disarm by itself")
        link.stream(0.3, True, target)
        time.sleep(1.0)
        servos, motors = sitl.values("servo"), sitl.values("motor")
        check("servos return to centre", all(servos.get(i) == CENTRE for i in range(SERVOS)), str(servos))
        check("motors return to min_command", all(motors.get(i) == MOTOR_MIN for i in range(MOTORS)), str(motors))
        time.sleep(1.2)
        check("board reports disarmed", link.board_armed is False)

        print("9. After link loss RC alone must not re-arm; an armed HEARTBEAT does")
        link.stream(1.0, None, target)
        check("RC without arm stays disarmed", sitl.values("servo").get(0) == CENTRE, str(sitl.values("servo")))
        link.stream(1.0, True, target)
        check("armed HEARTBEAT arms again", sitl.values("servo").get(0) == target[0], str(sitl.values("servo")))

        print("10. MAV_CMD_COMPONENT_ARM_DISARM")
        link.stream(0.3, False, target)           # disarm through the heartbeat flag
        check("disarmed again", sitl.values("servo").get(0) == CENTRE)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:             # arm by command only (no heartbeat)
            link.send(link.gen.arm_command(True))
            link.send(link.gen.rc_override(target))
            time.sleep(0.05)
        check("arm command arms", sitl.values("servo").get(0) == target[0], str(sitl.values("servo")))
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            link.send(link.gen.arm_command(False))
            link.send(link.gen.rc_override(target))
            time.sleep(0.05)
        check("disarm command centres the servos", sitl.values("servo").get(0) == CENTRE, str(sitl.values("servo")))
    finally:
        if link:
            link.close()
        sitl.stop()

    failed = results.count(False)
    print("\n%d checks, %d failed" % (len(results), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
