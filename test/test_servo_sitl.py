#!/usr/bin/env python3
"""Test the SATELLITEBOARD servo driver on SITL, no hardware needed.

  test/test_servo_sitl.py [--build] [--keep] [--verbose]

1. builds SITL with USE_SERVO_DRIVER into obj/main_servotest (separate from the stock SITL build)
2. applies test/servo_config.txt through the CLI on tcp://127.0.0.1:5761 and saves it
3. restarts SITL, connects to the MAVLink port (UART2, tcp://127.0.0.1:5762) and plays a UBlox on the
   GPS port (UART3, tcp://127.0.0.1:5763)
4. sends MAVLink 2 ADUM_AUX_CHANNELS frames like the master flight controller and checks the servo/motor
   outputs that SITL prints: servos follow always, motors only while armed, loss of communication zeroes the
   motors and holds the servos; UBX NAV-PVT frames must come back as ADUM_GPS_PVT

Uses only the Python standard library. Exit code 0 when every check passes.
"""
import argparse
import os
import datetime
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import mavlink_gen as mav  # noqa: E402

OBJ_DIR = os.path.join(ROOT, "obj", "main_servotest")
ELF = os.path.join(OBJ_DIR, "betaflight_SITL.elf")
RUN_DIR = os.path.join(HERE, "sitl-run-servotest")
CONFIG = os.path.join(HERE, "servo_config.txt")
CLI_PORT, MAVLINK_PORT, GPS_PORT = 5761, 5762, 5763
CONFIG_OVERRIDES = ["set gps_auto_config = OFF"]   # the fake module does not answer the setup commands

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
    commands[-1:-1] = CONFIG_OVERRIDES           # before the final "save"
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
    """MAVLink 2 connection to the board: the master side. Sends frames, decodes what the board answers."""

    def __init__(self, sock):
        self.sock = sock
        self.gen = mav.MavlinkGenerator()
        self.parser = mav.FrameParser()
        self.board_armed = None
        self.heartbeats = 0
        self.gps = []              # decoded ADUM_GPS_PVT messages
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
            for msgid, payload in self.parser.feed(chunk):
                if msgid == mav.MSG_HEARTBEAT:
                    self.board_armed = mav.decode_heartbeat(payload)["armed"]
                    self.heartbeats += 1
                elif msgid == mav.MSG_ADUM_GPS_PVT:
                    self.gps.append(mav.decode_gps_pvt(payload))

    def send(self, data):
        self.sock.sendall(data)

    def frames(self, seconds, armed, motors, servos, rate_hz=20, step=None):
        """Send ADUM_AUX_CHANNELS at rate_hz for `seconds`. step(fraction) may return (armed, motors, servos)."""
        begin = time.monotonic()
        end = begin + seconds
        while time.monotonic() < end:
            if step:
                armed, motors, servos = step((time.monotonic() - begin) / seconds)
            self.send(self.gen.aux_channels(armed, motors, servos))
            time.sleep(1.0 / rate_hz)

    def heartbeats_only(self, seconds, rate_hz=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(self.gen.heartbeat(True))
            time.sleep(1.0 / rate_hz)

    def close(self):
        self.stop = True
        self.sock.close()


def ubx_nav_pvt(**kw):
    """UBX NAV-PVT frame (92 byte payload) as sent by a UBlox module."""
    v = dict(itow=0, year=2026, month=10, day=26, hour=14, minute=30, sec=15, valid=0x07, tacc=50, nano=500000000,
             fix_type=3, flags=0x01, flags2=0, num_sv=17, lon=85432100, lat=473977000, height=477000, hmsl=430000,
             hacc=2500, vacc=3100, vel_n=1000, vel_e=-2000, vel_d=300, gspeed=2236, head_mot=12345678, sacc=450,
             head_acc=100000, pdop=123)
    v.update(kw)
    payload = struct.pack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHB5xihH", v["itow"], v["year"], v["month"], v["day"], v["hour"],
                          v["minute"], v["sec"], v["valid"], v["tacc"], v["nano"], v["fix_type"], v["flags"],
                          v["flags2"], v["num_sv"], v["lon"], v["lat"], v["height"], v["hmsl"], v["hacc"], v["vacc"],
                          v["vel_n"], v["vel_e"], v["vel_d"], v["gspeed"], v["head_mot"], v["sacc"], v["head_acc"],
                          v["pdop"], 0, 0, 0, 0)
    assert len(payload) == 92, len(payload)
    body = bytes([0x01, 0x07]) + struct.pack("<H", 92) + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


class FakeUblox:
    """Plays the GPS module on the GPS UART: repeats the current NAV-PVT frame at 5 Hz."""

    def __init__(self, sock):
        self.sock = sock
        self.frame = None
        self.stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.stop:
            if self.frame:
                try:
                    self.sock.sendall(self.frame)
                except OSError:
                    return
            time.sleep(0.2)

    def close(self):
        self.stop = True
        self.sock.close()


ADUM_TIME, ADUM_DIFF, ADUM_HEADING = (mav.ADUM_GPS_PVT_FLAGS_TIME_VALID, mav.ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS,
                                    mav.ADUM_GPS_PVT_FLAGS_HEADING_VALID)

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name, (" - " + detail) if detail else ""))


def expected_unix_usec(year, month, day, hour, minute, sec, nano):
    base = datetime.datetime(year, month, day, hour, minute, sec, tzinfo=datetime.timezone.utc)
    return int(base.timestamp()) * 1000000 + nano // 1000


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

    for port in (CLI_PORT, MAVLINK_PORT, GPS_PORT):
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.5).close()
        except OSError:
            continue
        print("tcp port %d is already in use: another SITL is running? (pkill -x betaflight_SITL)" % port)
        return 2

    sitl = Sitl(args.verbose)
    link = ublox = None
    try:
        print("1. Configure (servo_config.txt over the CLI)")
        sitl.start()
        if not sitl.wait_for("bind port %d" % CLI_PORT):
            check("SITL starts", False, "no CLI port; check %s" % ELF)
            return 1
        problems = apply_config(args.verbose)
        check("config accepted by the CLI", not problems, "; ".join(problems))
        time.sleep(1.0)
        sitl.stop()               # `save` already exits SITL, this only makes sure

        print("2. Restart with the saved config")
        sitl = Sitl(args.verbose)
        sitl.start()
        check("MAVLink port is open on UART2", sitl.wait_for("bind port %d" % MAVLINK_PORT),
              "serial port with the MAVLink function was not opened")
        check("GPS port is open on UART3", sitl.wait_for("bind port %d" % GPS_PORT))
        time.sleep(0.5)
        link = BoardLink(connect(MAVLINK_PORT))
        ublox = FakeUblox(connect(GPS_PORT))

        def servos():
            v = sitl.values("servo")
            return [v.get(i) for i in range(SERVOS)]

        def motors():
            v = sitl.values("motor")
            return [v.get(i) for i in range(MOTORS)]

        idle = [MOTOR_MIN] * MOTORS
        pose = [1800, 1200, 1700, 1300, 1600]
        run = [1500, 1600, 1700]

        print("3. Outputs after start-up")
        check("servos centred", servos() == [CENTRE] * SERVOS, str(servos()))
        check("motors at zero throttle (min_command)", motors() == idle, str(motors()))
        time.sleep(1.2)
        check("board sends a HEARTBEAT", link.heartbeats > 0)

        print("4. Disarmed: servos follow the frames, motors stay at zero")
        link.frames(1.0, False, [1400, 1400, 1400], pose)
        check("servos follow servo 1-5 although disarmed", servos() == pose, str(servos()))
        check("motors stay at zero", motors() == idle, str(motors()))

        print("5. Arming: the motor values of the armed frame are applied at once")
        link.send(link.gen.aux_channels(True, run, pose))
        time.sleep(0.3)
        check("motors follow the armed frame", motors() == run, str(motors()))
        link.frames(1.0, True, run, pose)
        check("board reports armed in its HEARTBEAT", link.board_armed is True)

        print("6. Servo 1 sweep 1000 -> 2000 (are the servos moving?)")
        since = time.monotonic()
        link.frames(2.0, True, run, pose, step=lambda f: (True, run, [int(1000 + 1000 * f)] + pose[1:]))
        sweep = sitl.changes("servo", 0, since)
        check("servo 1 changes continuously", len(set(sweep)) >= 20, "%d distinct values" % len(set(sweep)))
        check("servo 1 sweep is monotonic", sweep == sorted(sweep), "first %s last %s" % (sweep[:1], sweep[-1:]))
        check("servo 1 reaches the end of the range", bool(sweep) and sweep[-1] >= 1950, "last %s" % sweep[-1:])

        print("7. Clamping, servos 6-8 and invalid values (servo limits are not applied, only 500-2500)")
        link.frames(0.5, True, [3000, 0, None], [3000, 100, None, 0, 1500, 1111, 1111, 1111])
        check("servo pulse clamped to 2500 / 500", servos()[:2] == [2500, 500], str(servos()))
        check("servo values 0 / 65535 are ignored", servos()[2:4] == pose[2:4], str(servos()))
        check("motor clamped to max_throttle, invalid motor values go to zero", motors() == [MOTOR_MAX, MOTOR_MIN, MOTOR_MIN], str(motors()))

        print("8. Disarm by the master: motors to zero at once, servos still follow")
        link.send(link.gen.aux_channels(False, run, pose))
        time.sleep(0.3)
        check("motors at zero", motors() == idle, str(motors()))
        check("servos follow the frame", servos() == pose, str(servos()))

        print("9. Lost communication while armed: motors to zero, servos hold the last frame")
        link.frames(0.5, True, run, pose)
        check("motors running", motors() == run, str(motors()))
        time.sleep(1.0)                           # total silence
        check("motors pulled to zero", motors() == idle, str(motors()))
        check("servos stay where the last frame put them", servos() == pose, str(servos()))
        time.sleep(1.2)
        check("board reports disarmed", link.board_armed is False)

        print("10. After the link is back: a disarmed frame moves the servos only, an armed one runs the motors")
        link.frames(0.5, False, run, [1100, 1900, 1500, 1500, 1500])
        check("servos follow", servos()[:2] == [1100, 1900], str(servos()))
        check("motors stay at zero", motors() == idle, str(motors()))
        link.frames(0.5, True, run, [1100, 1900, 1500, 1500, 1500])
        check("motors follow the armed frame", motors() == run, str(motors()))

        print("11. HEARTBEAT counts as communication")
        link.heartbeats_only(1.5)
        check("board stays armed on heartbeats alone", motors() == run, str(motors()))

        print("12. ADUM_GPS_PVT from UBX NAV-PVT")
        cases = [
            ("3D fix", dict(), dict(fix_type=3, flags=0x01, sat=17, flags_out=ADUM_TIME | ADUM_HEADING)),
            ("DGPS", dict(lat=473977001, flags=0x03), dict(fix_type=4, flags_out=ADUM_TIME | ADUM_DIFF | ADUM_HEADING)),
            ("RTK float", dict(lat=473977002, flags=0x03 | (1 << 6)), dict(fix_type=5, flags_out=ADUM_TIME | ADUM_DIFF | ADUM_HEADING)),
            ("RTK fixed", dict(lat=473977003, flags=0x03 | (2 << 6)), dict(fix_type=6, flags_out=ADUM_TIME | ADUM_DIFF | ADUM_HEADING)),
            ("2D fix (heading needs a 3D fix)", dict(lat=473977004, fix_type=2), dict(fix_type=2, flags_out=ADUM_TIME)),
            ("no fix", dict(lat=473977005, fix_type=0, flags=0x00), dict(fix_type=1, flags_out=ADUM_TIME)),
            ("slow, time and DOP unknown", dict(lat=473977006, gspeed=100, valid=0, pdop=0),
             dict(fix_type=3, flags_out=0, pdop=0xFFFF, time=0)),
        ]
        for name, pvt_args, want in cases:
            pvt_args = dict(pvt_args)
            lat = pvt_args.setdefault("lat", 473977000)
            ublox.frame = ubx_nav_pvt(**pvt_args)
            deadline = time.monotonic() + 3.0
            got = None
            while time.monotonic() < deadline and got is None:
                got = next((g for g in reversed(link.gps) if g["lat"] == lat), None)
                time.sleep(0.05)
            if got is None:
                check("%s: ADUM_GPS_PVT received" % name, False, "none in 3 s")
                continue
            ok = (got["fix_type"] == want["fix_type"] and got["flags"] == want["flags_out"]
                  and got["satellites_visible"] == 17 and got["lon"] == 85432100
                  and got["alt_msl"] == 430000 and got["alt_ellipsoid"] == 477000
                  and (got["vel_n"], got["vel_e"], got["vel_d"]) == (1000, -2000, 300)
                  and got["h_acc"] == 2500 and got["v_acc"] == 3100 and got["s_acc"] == 450
                  and got["heading_motion"] == 12345678 and got["pdop"] == want.get("pdop", 123)
                  and got["time_usec"] == want.get("time", expected_unix_usec(2026, 10, 26, 14, 30, 15, 500000000)))
            check("%s -> fix_type %d, flags %d" % (name, want["fix_type"], want["flags_out"]), ok, str(got))
    finally:
        for item in (link, ublox):
            if item:
                item.close()
        sitl.stop()

    failed = results.count(False)
    print("\n%d checks, %d failed" % (len(results), failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
