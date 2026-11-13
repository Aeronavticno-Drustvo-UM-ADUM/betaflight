"""Shared harness of the SATELLITEBOARD servo driver SITL tests (standard library only).

Classes: Sitl (runs the binary, tracks the [servo]/[motor] lines), BoardLink (the master side of the MAVLink
port), FakeUblox (GPS module on the GPS port). run_scenarios() builds if needed, configures a fresh SITL,
connects and runs the scenario functions of a test file; check() / known_issue() / finish() report.
"""
import argparse
import datetime
import os
import random
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.dirname(HERE)
ROOT = os.path.dirname(TEST_DIR)
sys.path.insert(0, HERE)
import mavlink_gen as mav  # noqa: E402

OBJ_DIR = os.path.join(ROOT, "obj", "main_servotest")
ELF = os.path.join(OBJ_DIR, "betaflight_SITL.elf")
RUN_DIR = os.path.join(TEST_DIR, "sitl-run-servotest")
CONFIG = os.path.join(TEST_DIR, "servo_config.txt")
CLI_PORT, MAVLINK_PORT, GPS_PORT = 5761, 5762, 5763
CONFIG_OVERRIDES = ["set gps_auto_config = OFF"]   # the fake module does not answer the setup commands

CENTRE, MOTOR_MIN, MOTOR_MAX = 1500, 1000, 2000   # servo_center_pulse, min_command, max_throttle in servo_config.txt
SERVOS, MOTORS = 5, 3
FAILSAFE_TIMEOUT_S = 0.5                           # servo_driver.c: no ADUM_AUX_CHANNELS for 500 ms -> disarm
IDLE = [MOTOR_MIN] * MOTORS
POSE = [1800, 1200, 1700, 1300, 1600]              # servo pulses in us, avoid bytes 0xFD in the little-endian words
RUN = [1500, 1600, 1700]                           # motor pulses in us

# ---------------------------------------------------------------- reporting
_results = []      # (name, ok)
_known = []


def scenario(number, title):
    print("%d. %s" % (number, title), flush=True)


def check(name, ok, detail=""):
    ok = bool(ok)
    _results.append((name, ok))
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name, (" - " + detail) if detail else ""), flush=True)
    return ok


def known_issue(name, observed):
    """Documents current behaviour that is an open design question; never fails the run."""
    _known.append(name)
    print("  [KNOWN] %s - observed: %s" % (name, observed), flush=True)


def finish():
    failed = sum(1 for _, ok in _results if not ok)
    print("\nSUMMARY checks=%d failed=%d known=%d" % (len(_results), failed, len(_known)), flush=True)
    return 1 if failed else 0


# ---------------------------------------------------------------- SITL
class Sitl:
    """Runs the SITL binary and tracks the [servo]/[motor] lines it prints (one line per change)."""

    def __init__(self, verbose):
        self.verbose = verbose
        self.proc = None
        self.servo = {}
        self.motor = {}
        self.history = []          # (time.monotonic(), "[servo]" | "[motor]", index, value)
        self.rate_log = []         # (time.monotonic(), motor writes in the last second) from the [motorrate] hook
        self.t_start = None        # time.monotonic() when the process was started (boot reference)
        self.lines = []
        self.lock = threading.Lock()

    def start(self):
        os.makedirs(RUN_DIR, exist_ok=True)
        self.t_start = time.monotonic()
        self.proc = subprocess.Popen(["stdbuf", "-o0", "-e0", ELF], cwd=RUN_DIR, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        for line in self.proc.stdout:
            line = line.rstrip()
            with self.lock:
                self.lines.append(line)
                if line.startswith("[motorrate]"):
                    self.rate_log.append((time.monotonic(), int(line[len("[motorrate]"):])))
                    continue
                for kind, table in (("[servo]", self.servo), ("[motor]", self.motor)):
                    if line.startswith(kind):
                        index, value = line[len(kind):].split("=")
                        table[int(index)] = int(value)
                        self.history.append((time.monotonic(), kind, int(index), int(value)))
            if self.verbose and not line.startswith("[FLASH_"):
                print("   sitl|", line)

    def rates(self, since, until=float("inf")):
        with self.lock:
            return [n for t, n in self.rate_log if since <= t <= until]

    def time_jump(self):
        """SITL only: moves the SITL clock forward by 25 min (SIGUSR1) and waits for the [SITL] time jump line."""
        self.proc.send_signal(signal.SIGUSR1)
        time.sleep(0.1)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

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

    def count(self, text):
        with self.lock:
            return sum(1 for l in self.lines if text in l)

    def accepts(self, port, timeout=3.0):
        """Probes that SITL's tcp thread really accepts on port (a listener registered while the dyad thread was
        running can be lost, or the process crashes at start-up): connects, waits for the [NEW] line, disconnects and
        waits for [CLS], so the port is free again."""
        uart = "UART%d" % (port - CLI_PORT + 1)
        new, closed = self.count("[NEW]" + uart), self.count("[CLS]" + uart)
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=2)
        except OSError:
            return False
        end = time.monotonic() + timeout
        while self.count("[NEW]" + uart) == new and time.monotonic() < end and self.alive():
            time.sleep(0.02)
        accepted = self.count("[NEW]" + uart) > new
        sock.close()
        while accepted and self.count("[CLS]" + uart) == closed and time.monotonic() < end + 2.0:
            time.sleep(0.02)
        return accepted

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()

    def values(self, kind):
        with self.lock:
            return dict(self.servo if kind == "servo" else self.motor)

    def events(self, kind, index, since=0.0, until=float("inf")):
        """[(time, value)] of the changes of one output in [since, until]."""
        tag = "[%s]" % kind
        with self.lock:
            return [(t, v) for (t, k, i, v) in self.history if since <= t <= until and k == tag and i == index]

    def changes(self, kind, index, since, until=float("inf")):
        return [v for _, v in self.events(kind, index, since, until)]

    def seen(self, kind, index, since, until=float("inf")):
        """Every value the output had in [since, until]: the value held at `since` plus all changes."""
        tag = "[%s]" % kind
        with self.lock:
            before = [v for (t, k, i, v) in self.history if t < since and k == tag and i == index]
        values = set(before[-1:])
        values.update(self.changes(kind, index, since, until))
        return values


# ---------------------------------------------------------------- tcp helpers, CLI
def connect(port, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=2)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            return sock
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


def cli_session(timeout=3.0):
    """Connects to the CLI port and enters the CLI. Returns the socket, or None when no banner came."""
    sock = connect(CLI_PORT)
    sock.sendall(b"#")
    if "CLI" in read_until(sock, b"# ", timeout):
        return sock
    sock.close()
    return None


def apply_config(verbose, sock):
    """Send servo_config.txt through the CLI session sock (from start_with_cli). The last line (save) restarts SITL. Returns the list of problems."""
    commands = []
    with open(CONFIG) as f:
        for raw in f:
            line = raw.strip()
            if line and not line.startswith("#"):
                commands.append(line)
    commands[-1:-1] = CONFIG_OVERRIDES           # before the final "save"
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


def build():
    print("Building SITL with USE_SERVO_DRIVER ...", flush=True)
    # SITL uses the host gcc: dummy GCC_VERSION/GCC_REQUIRED_VERSION skip the arm-none-eabi-gcc check
    subprocess.run(["make", "-C", ROOT, "TARGET=SITL", "GCC_VERSION=none", "GCC_REQUIRED_VERSION=none",
                    "OBJECT_DIR=" + OBJ_DIR, "EXTRA_FLAGS=-DUSE_SERVO_DRIVER",
                    "-j%d" % os.cpu_count()], check=True, stdout=subprocess.DEVNULL)


def ports_in_use():
    busy = []
    for port in (CLI_PORT, MAVLINK_PORT, GPS_PORT):
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.5).close()
        except OSError:
            continue
        busy.append(port)
    return busy


# ---------------------------------------------------------------- frames
def raw_frame_v2(msgid, payload, crc_extra, sysid=1, compid=1, seq=0, incompat=0, signature=b""):
    """MAVLink 2 frame as given: payload not trimmed, any CRC_EXTRA, optional signature bytes after the CRC."""
    header = struct.pack("<BBBBBB", len(payload), incompat, 0, seq, sysid, compid) + msgid.to_bytes(3, "little")
    crc = mav.mavlink_crc(header + payload, crc_extra)
    return bytes([mav.STX_V2]) + header + payload + struct.pack("<H", crc) + signature


def raw_frame_v1(msgid, payload, crc_extra, sysid=1, compid=1, seq=0):
    header = struct.pack("<BBBBB", len(payload), seq, sysid, compid, msgid)
    crc = mav.mavlink_crc(header + payload, crc_extra)
    return b"\xfe" + header + payload + struct.pack("<H", crc)


def aux_payload(armed, motors, servos, arming_raw=None):
    """Untrimmed ADUM_AUX_CHANNELS payload (23 bytes)."""
    pad = lambda values, n: [0xFFFF if v is None else int(v) for v in values] + [0xFFFF] * (n - len(values))  # noqa: E731
    status = arming_raw if arming_raw is not None else (1 if armed else 0)
    return struct.pack("<3H8HB", *pad(motors, 3), *pad(servos, 8), status)


def corrupt_crc(frame):
    return frame[:-1] + bytes([frame[-1] ^ 0xFF])


class BoardLink:
    """MAVLink 2 connection to the board: the master side. Sends frames, decodes what the board answers."""

    def __init__(self, sock):
        self.sock = sock
        self.gen = mav.MavlinkGenerator()
        self.parser = mav.FrameParser()
        self.hb_log = []           # (time.monotonic(), armed) of every HEARTBEAT received
        self.hb_full = []          # (time.monotonic(), armed, system_status) of every HEARTBEAT received
        self.texts = []            # (time.monotonic(), severity, text) of every STATUSTEXT received
        self.sys = []              # (time.monotonic(), decoded SYS_STATUS) of every SYS_STATUS received
        self.gps = []              # decoded ADUM_GPS_PVT messages
        self.last_aux_t = None     # time.monotonic() just before the last ADUM_AUX_CHANNELS was written
        self.stop = False
        threading.Thread(target=self._reader, daemon=True).start()

    @property
    def board_armed(self):
        return self.hb_log[-1][1] if self.hb_log else None

    @property
    def heartbeats(self):
        return len(self.hb_log)

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
                    hb = mav.decode_heartbeat(payload)
                    now = time.monotonic()
                    self.hb_full.append((now, hb["armed"], hb["system_status"]))
                    self.hb_log.append((now, hb["armed"]))
                elif msgid == mav.MSG_STATUSTEXT:
                    st = mav.decode_statustext(payload)
                    self.texts.append((time.monotonic(), st["severity"], st["text"]))
                elif msgid == mav.MSG_SYS_STATUS:
                    self.sys.append((time.monotonic(), mav.decode_sys_status(payload)))
                elif msgid == mav.MSG_ADUM_GPS_PVT:
                    self.gps.append(mav.decode_gps_pvt(payload))

    def send(self, data):
        self.sock.sendall(data)

    def send_aux(self, armed, motors, servos, arming_raw=None):
        """One ADUM_AUX_CHANNELS frame; returns the time it was written."""
        frame = self.gen.aux_channels(armed, motors, servos, arming_raw)
        t = time.monotonic()
        self.sock.sendall(frame)
        self.last_aux_t = t
        return t

    def send_aux_frame(self, frame):
        """A prepared frame that counts as the last ADUM_AUX_CHANNELS (when valid)."""
        t = time.monotonic()
        self.sock.sendall(frame)
        self.last_aux_t = t
        return t

    def frames(self, seconds, armed, motors, servos, rate_hz=50, step=None, arming_raw=None):
        """ADUM_AUX_CHANNELS at rate_hz for `seconds`. step(fraction) may return (armed, motors, servos)."""
        period = 1.0 / rate_hz
        begin = time.monotonic()
        end = begin + seconds
        nxt = begin
        while time.monotonic() < end:
            if step:
                armed, motors, servos = step((time.monotonic() - begin) / seconds)
            self.send_aux(armed, motors, servos, arming_raw)
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    def stream_until_heartbeat(self, armed, motors, servos, timeout=2.0, arming_raw=None):
        """Keep sending frames at 50 Hz until a new HEARTBEAT arrives; returns its armed flag (None: timeout)."""
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self.frames(0.1, armed, motors, servos, arming_raw=arming_raw)
            for t, flag in list(self.hb_log):
                if t > start:
                    return flag
        return None

    def gap(self, seconds):
        """Sleep until `seconds` after the last ADUM_AUX_CHANNELS."""
        delay = self.last_aux_t + seconds - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    def heartbeats_only(self, seconds, rate_hz=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(self.gen.heartbeat(True))
            time.sleep(1.0 / rate_hz)

    def wait_heartbeat(self, after, timeout=2.5):
        """armed flag of the first HEARTBEAT received later than `after` (monotonic), None if none."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for t, armed in list(self.hb_log):
                if t > after:
                    return armed
            time.sleep(0.02)
        return None

    def status_after(self, after, timeout=2.5, keep=None):
        """system_status of the first HEARTBEAT later than `after`; None if none. keep() is called while waiting
        (e.g. to send frames) when given."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for t, _, status in list(self.hb_full):
                if t > after:
                    return status
            if keep:
                keep()
            else:
                time.sleep(0.02)
        return None

    def texts_since(self, since, prefix="", until=float("inf")):
        """[(time, severity, text)] of the STATUSTEXTs starting with `prefix` in [since, until]."""
        return [x for x in list(self.texts) if since <= x[0] <= until and x[2].startswith(prefix)]

    def wait_text(self, prefix, since, timeout=1.0, keep=None):
        """First STATUSTEXT starting with `prefix` later than `since` as (time, severity, text); None on timeout.
        keep() is called while waiting (e.g. to keep sending frames) when given."""
        end = time.monotonic() + timeout
        while True:
            found = self.texts_since(since, prefix)
            if found:
                return found[0]
            if time.monotonic() >= end:
                return None
            if keep:
                keep()
            else:
                time.sleep(0.01)

    def sys_after(self, after, timeout=2.0, keep=None):
        """First SYS_STATUS (time, dict) received later than `after`; None on timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for t, d in list(self.sys):
                if t > after:
                    return t, d
            if keep:
                keep()
            else:
                time.sleep(0.02)
        return None

    def heartbeats_between(self, since, until):
        return [(t, a) for t, a in list(self.hb_log) if since <= t <= until]

    def close(self):
        self.stop = True
        try:
            self.sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------- GPS module
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


def expected_unix_usec(year, month, day, hour, minute, sec, nano):
    base = datetime.datetime(year, month, day, hour, minute, sec, tzinfo=datetime.timezone.utc)
    return int(base.timestamp()) * 1000000 + nano // 1000


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


# ---------------------------------------------------------------- fixture
class Fixture:
    """A configured SITL with a MAVLink link (and a fake GPS): what a scenario works with."""

    def __init__(self, sitl, link, ublox):
        self.sitl, self.link, self.ublox = sitl, link, ublox

    def servos(self):
        v = self.sitl.values("servo")
        return [v.get(i) for i in range(SERVOS)]

    def motors(self):
        v = self.sitl.values("motor")
        return [v.get(i) for i in range(MOTORS)]

    def settle(self, armed=False, motors=RUN, servos=POSE, seconds=0.4):
        """Bring the board into a known state: constant frames at 50 Hz."""
        self.link.frames(seconds, armed, motors, servos)

    def wait_outputs(self, servos=None, motors=None, timeout=0.5):
        """Poll until the outputs equal the expected values (None = not checked); returns the wait in s or None."""
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            if (servos is None or self.servos() == list(servos)) and (motors is None or self.motors() == list(motors)):
                return time.monotonic() - begin
            time.sleep(0.002)
        return None

    def disarm_times(self, since, until=float("inf")):
        """Time (monotonic) at which each motor first dropped to min_command after `since`; None if it did not."""
        times = []
        for i in range(MOTORS):
            hits = [t for t, v in self.sitl.events("motor", i, since, until) if v == MOTOR_MIN]
            times.append(hits[0] if hits else None)
        return times

    def restart(self, wait_exit=True):
        """Stops SITL, starts it again (the eeprom is kept) and reconnects the MAVLink link. sitl.t_start is the boot
        reference. Returns False when the MAVLink port did not open."""
        self.link.close()
        if self.sitl.alive() and wait_exit:
            self.sitl.stop()
        verbose = self.sitl.verbose
        self.sitl.stop()
        time.sleep(1.0)
        sitl, (ok,) = start_with_ports(verbose, (MAVLINK_PORT,))
        self.sitl = sitl
        if ok:
            self.link = BoardLink(connect(MAVLINK_PORT))
        return ok

    def time_jump(self):
        self.sitl.time_jump()

    def since_boot(self):
        return time.monotonic() - self.sitl.t_start

    def reconnect_link(self):
        self.link.close()
        self.link = BoardLink(connect(MAVLINK_PORT))
        return self.link


def parse_args(doc):
    ap = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", action="store_true", help="rebuild the SITL test binary first")
    ap.add_argument("--keep", action="store_true", help="keep the SITL config between runs")
    ap.add_argument("--verbose", action="store_true", help="print the CLI dialogue and the SITL log")
    return ap.parse_args()


def start_with_ports(verbose, ports, attempts=4):
    """Starts SITL and waits until it reports the tcp ports. SITL sometimes fails to bind right after the previous
    instance exited (TIME_WAIT): then it is killed and started again. Returns (sitl, [port opened, ...])."""
    for attempt in range(attempts):
        sitl = Sitl(verbose)
        sitl.start()
        opened = [sitl.wait_for("bind port %d" % port, 5.0) for port in ports]
        if all(opened) and all(sitl.accepts(port) for port in ports):
            break
        opened = [False] * len(ports) if all(opened) else opened
        with sitl.lock:
            tail = [l for l in sitl.lines if not l.startswith("[")][-4:]
        print("   SITL did not open tcp port(s) %s (attempt %d, alive=%s, rc=%s), restarting; last lines: %s"
              % (list(ports), attempt + 1, sitl.alive(), sitl.proc.poll(), tail), flush=True)
        sitl.stop()
        time.sleep(3.0)
    return sitl, opened


def start_with_cli(verbose, attempts=5):
    """Starts SITL until the CLI answers '#' with its banner. Sometimes SITL binds the CLI port but its tcp thread
    never accepts the connection (seen: no accept for > 8 s, process alive, connection silently queued): the instance
    is then killed and started again. Returns (sitl, CLI socket already in the CLI) or (sitl, None)."""
    for attempt in range(attempts):
        sitl, (opened,) = start_with_ports(verbose, (CLI_PORT,))
        if opened:
            sock = cli_session()
            if sock is not None:
                return sitl, sock
        print("   SITL CLI did not answer (attempt %d), restarting" % (attempt + 1), flush=True)
        sitl.stop()
        time.sleep(1.0)
    return sitl, None


def run_scenarios(args, scenarios, ublox=False):
    """Configures a fresh SITL and calls every scenario(fixture); the docstring's first line is its title.
    Returns the exit code: 0 all checks pass, 1 a check failed, 2 the setup is not possible."""
    if args.build or not os.path.exists(ELF):
        build()
    busy = ports_in_use()
    if busy:
        print("tcp port(s) %s already in use: another SITL is running? (pkill -x betaflight_SITL)" % busy)
        return 2
    if not args.keep and os.path.exists(os.path.join(RUN_DIR, "eeprom.bin")):
        os.remove(os.path.join(RUN_DIR, "eeprom.bin"))

    sitl = Sitl(args.verbose)
    fx = None
    try:
        print("0. Setup (servo_config.txt over the CLI, restart)", flush=True)
        sitl, cli = start_with_cli(args.verbose)
        if cli is None:
            check("SITL starts", False, "no CLI port or banner; check %s" % ELF)
            return finish()
        problems = apply_config(args.verbose, cli)
        check("config accepted by the CLI", not problems, "; ".join(problems))
        time.sleep(1.0)
        sitl.stop()               # `save` already exits SITL, this only makes sure
        sitl, (mavlink_ok, gps_ok) = start_with_ports(args.verbose, (MAVLINK_PORT, GPS_PORT))
        check("MAVLink port is open on UART2", mavlink_ok)
        check("GPS port is open on UART3", gps_ok)
        if not (mavlink_ok and gps_ok):
            return finish()
        time.sleep(0.5)
        fx = Fixture(sitl, BoardLink(connect(MAVLINK_PORT)), FakeUblox(connect(GPS_PORT)) if ublox else None)
        for number, func in enumerate(scenarios, 1):
            title = (func.__doc__ or func.__name__).strip().splitlines()[0]
            scenario(number, title)
            try:
                func(fx)
            except Exception:  # noqa: BLE001 - a crashed scenario is a failed check, the others still run
                traceback.print_exc()
                check("scenario %d ran to the end" % number, False, "exception, see traceback")
            if not fx.sitl.alive():
                check("SITL still running after scenario %d" % number, False)
                break
    finally:
        if fx:
            fx.link.close()
            if fx.ublox:
                fx.ublox.close()
            fx.sitl.stop()          # may be a restarted instance
        sitl.stop()
    return finish()


def median(values):
    ordered = sorted(values)
    n = len(ordered)
    return ordered[n // 2] if n % 2 else 0.5 * (ordered[n // 2 - 1] + ordered[n // 2])


def rng(seed):
    return random.Random(seed)
