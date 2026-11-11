#!/usr/bin/env python3
"""Send MAVLink 2.0 ADUM_AUX_CHANNELS to a SATELLITEBOARD over a serial port, to check it with the real board.

  test/send_aux_channels.py [PORT] [options]

Default: a 0.5 Hz sine sweep of servos 1-5 between 1000 and 2000 us (each servo shifted in phase),
sent at 50 Hz, board DISARMED and motors at 1000 us. Press Ctrl-C to stop.

  --arm               send arming_status = ARMED (the motors then follow --motors)
  --motors 1100,1100,1100   motor pulses in us; they only take effect with --arm
  --port /dev/ttyUSB1 --baud 115200 --freq 0.5 --min 1000 --max 2000 --rate 50 --servos 5 --duration 10

The board answers with a HEARTBEAT (arm state) and ADUM_GPS_PVT; both are printed. Standard library only
(Linux/macOS serial through termios). Without an armed flag the motors never run. Remove propellers and
disconnect loads before using --arm.
"""
import argparse
import math
import os
import sys
import termios
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mavlink_gen as mav  # noqa: E402

BAUDS = {
    9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400, 57600: termios.B57600,
    115200: termios.B115200, 230400: termios.B230400,
}
MOTOR_COUNT = 3
SERVO_SLOTS = 8


def open_port(path, baud):
    if baud not in BAUDS:
        raise SystemExit("unsupported baud %d, use one of %s" % (baud, ", ".join(str(b) for b in BAUDS)))
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        hint = ""
        if e.errno == 13:
            hint = " (permission denied: add yourself to the dialout group, then log in again)"
        elif e.errno == 2:
            hint = " (no such device: check the cable, `ls /dev/ttyUSB* /dev/ttyACM*`)"
        raise SystemExit("cannot open %s: %s%s" % (path, e.strerror, hint))
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                            # iflag: no input processing
    attrs[1] = 0                                            # oflag: raw output
    attrs[2] = (termios.CS8 | termios.CREAD | termios.CLOCAL)   # 8N1, no flow control
    attrs[3] = 0                                            # lflag: raw
    attrs[4] = attrs[5] = BAUDS[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def write_all(fd, data):
    while data:
        try:
            n = os.write(fd, data)
        except BlockingIOError:
            time.sleep(0.001)
            continue
        data = data[n:]


def read_available(fd):
    try:
        return os.read(fd, 4096)
    except BlockingIOError:
        return b""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port_pos", nargs="?", metavar="PORT", help="serial device, e.g. /dev/ttyUSB0")
    ap.add_argument("--port", default=None, help="serial device (default /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=115200, help="serial baud rate (default 115200)")
    ap.add_argument("--rate", type=float, default=50.0, help="frames per second (default 50)")
    ap.add_argument("--freq", type=float, default=0.5, help="sweep frequency in Hz (default 0.5)")
    ap.add_argument("--min", dest="lo", type=int, default=1000, help="sweep minimum in us (default 1000)")
    ap.add_argument("--max", dest="hi", type=int, default=2000, help="sweep maximum in us (default 2000)")
    ap.add_argument("--servos", type=int, default=5, help="number of servos to sweep, 1-8 (default 5)")
    ap.add_argument("--same-phase", action="store_true", help="move all servos together instead of shifted in phase")
    ap.add_argument("--arm", action="store_true", help="send ARMED (motors follow --motors)")
    ap.add_argument("--motors", default="1000,1000,1000", help="motor pulses in us, 3 values (default 1000,1000,1000)")
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (default: until Ctrl-C)")
    args = ap.parse_args()

    port = args.port_pos or args.port or "/dev/ttyUSB0"
    motors = [int(v) for v in args.motors.split(",")]
    if len(motors) != MOTOR_COUNT:
        raise SystemExit("--motors needs %d values" % MOTOR_COUNT)
    if not 1 <= args.servos <= SERVO_SLOTS:
        raise SystemExit("--servos must be 1-%d" % SERVO_SLOTS)
    if args.lo > args.hi:
        raise SystemExit("--min must not be above --max")

    fd = open_port(port, args.baud)
    gen = mav.MavlinkGenerator()
    parser = mav.FrameParser()
    centre, amplitude = (args.lo + args.hi) / 2.0, (args.hi - args.lo) / 2.0

    print("%s @ %d baud, %s, %.2f Hz sweep %d-%d us on servos 1-%d, %d frames/s%s" % (
        port, args.baud, "ARMED" if args.arm else "disarmed", args.freq, args.lo, args.hi, args.servos, args.rate,
        ", motors %s" % motors if args.arm else ""))
    if args.arm:
        print("WARNING: armed, motors will run at %s us" % motors)
    print("Ctrl-C to stop\n")

    begin = time.monotonic()
    next_frame = begin
    next_report = begin + 1.0
    sent = 0
    board_armed = None
    heartbeats = 0
    last_gps = None
    servos = [None] * SERVO_SLOTS
    try:
        while True:
            now = time.monotonic()
            t = now - begin
            if args.duration and t >= args.duration:
                break
            if now >= next_frame:
                for i in range(args.servos):
                    phase = 0.0 if args.same_phase else 2 * math.pi * i / args.servos
                    servos[i] = round(centre + amplitude * math.sin(2 * math.pi * args.freq * t + phase))
                write_all(fd, gen.aux_channels(args.arm, motors, servos))
                sent += 1
                next_frame += 1.0 / args.rate
                if next_frame < now - 1.0:           # fell far behind (suspended): do not burst
                    next_frame = now
            for msgid, payload in parser.feed(read_available(fd)):
                if msgid == mav.MSG_HEARTBEAT:
                    board_armed = mav.decode_heartbeat(payload)["armed"]
                    heartbeats += 1
                elif msgid == mav.MSG_ADUM_GPS_PVT:
                    last_gps = mav.decode_gps_pvt(payload)
            if now >= next_report:
                next_report += 1.0
                line = "t=%5.1fs sent=%5d servos=%s" % (t, sent, [s for s in servos[:args.servos]])
                line += " | board: " + ("no reply" if not heartbeats else "heartbeat %s" % ("ARMED" if board_armed else "disarmed"))
                if last_gps:
                    line += ", GPS fix=%d sats=%d lat=%.6f lon=%.6f" % (
                        last_gps["fix_type"], last_gps["satellites_visible"], last_gps["lat"] / 1e7, last_gps["lon"] / 1e7)
                print(line, flush=True)
            time.sleep(0.001)
    except KeyboardInterrupt:
        print()
    finally:
        # leave the board disarmed with the servos where they are: a few disarmed frames, then silence
        for _ in range(5):
            write_all(fd, gen.aux_channels(False, [mav.PULSE_IGNORE] * MOTOR_COUNT, servos))
            time.sleep(0.02)
        os.close(fd)
        print("stopped, sent %d frames%s" % (sent, "" if heartbeats else ", the board never answered (wrong port, baud or MAVLink port setting?)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
