"""Bench helpers for the standalone satellite hardware tests.

Setup: an FTDI adapter on the satellite UART1 acts as the fake master, an ST-Link (SWD) gives the ground truth.
The real master (EVIS) must NOT be connected to the satellite link.
Imports the SWD/OpenOCD and frame code from test/tools; nothing is copied.
"""
import os
import subprocess
import sys
import time

HW_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(os.path.dirname(HW_DIR))
TOOLS_DIR = os.path.join(REPO_DIR, "test", "tools")
sys.path.insert(0, TOOLS_DIR)
sys.path.insert(0, os.path.join(REPO_DIR, "test", "common"))
import hw_output_check as hw  # noqa: E402  (FrameStreamer, pulses, verify, symbol lookup)
import read_diag  # noqa: E402  (FIELDS, read_words)
import send_aux_channels as serial_io  # noqa: E402  (open_port)

FTDI_PORT = "/dev/serial/by-id/usb-FTDI_Dual_RS232-HS-if01-port0"
STLINK_USB_IDS = ("0483:3748", "0483:374b", "0483:374e", "0483:374f", "0483:3753", "0483:3754")  # V2, V2-1, V3 variants
MASTER_BAUD = 115200
OPENOCD_ARGS = "-f interface/stlink.cfg -f target/stm32f1x.cfg".split()
ELF_PATH = os.path.join(REPO_DIR, "obj", "main", "betaflight_SATELLITEBOARD.elf")

STATE_DISARMED, STATE_ARMED, STATE_FAILSAFE = hw.STATE_DISARMED, hw.STATE_ARMED, hw.STATE_FAILSAFE
STATE_NAMES = hw.STATE_NAMES
FAILSAFE_DELAY_MAX_MS = 10_000     # a delay above this is a stale measurement, not a failsafe delay
SILENCE_PROBE_S = 1.0              # FTDI silent time used to detect a still-connected EVIS


def read_diag_dict():
    """All servoDriverDiag fields over SWD (no halt) as {name: value}."""
    address = read_diag.find_symbol_address(ELF_PATH)
    words = read_diag.read_words(address, len(read_diag.FIELDS), " ".join(OPENOCD_ARGS))
    return dict(zip(read_diag.FIELDS, words))


def diag():
    """Current satellite diag block."""
    return read_diag_dict()


def stlink_present():
    """True when an ST-Link shows up on USB (lsusb, no OpenOCD session is opened)."""
    try:
        listing = subprocess.run(["lsusb"], capture_output=True, text=True).stdout
    except OSError:
        return False
    return any(usb_id in listing for usb_id in STLINK_USB_IDS)


def read_last_aux_us():
    """Raw lastAuxUs (32-bit microsecond timestamp of the last accepted AUX frame) over SWD."""
    address = hw.find_symbol_address("nm", ELF_PATH, "lastAuxUs")
    return read_diag.read_words(address, 1, " ".join(OPENOCD_ARGS))[0]


def failsafe_delay_ms():
    """Failsafe delay in ms = lastTransitionMs*1000 - lastAuxUs (signed 32-bit, wrap safe).

    Valid only when the board is in FAILSAFE (state 2); returns None otherwise or when the value is >= 10 s.
    """
    fields = read_diag_dict()
    if fields["lastToState"] != STATE_FAILSAFE:
        return None
    last_aux_us = read_last_aux_us()
    difference_us = (fields["lastTransitionMs"] * 1000 - last_aux_us) & 0xFFFFFFFF
    if difference_us >= 0x80000000:
        difference_us -= 0x100000000       # signed 32-bit
    delay_ms = difference_us / 1000.0
    if abs(delay_ms) >= FAILSAFE_DELAY_MAX_MS:
        return None
    return delay_ms


def motor_pulses_us():
    """Motor pin pulse widths in us {pin: us} (from the timer CCR registers over SWD)."""
    pulses, _moe = hw.read_pulses_us(OPENOCD_ARGS)
    return {pin: pulses[pin] for pin in hw.MOTOR_PINS}


def servo_pulses_us():
    """Servo pin pulse widths in us {pin: us}."""
    pulses, _moe = hw.read_pulses_us(OPENOCD_ARGS)
    return {pin: pulses[pin] for pin in hw.SERVO_PINS}


def pin_config():
    """Pin-level output config over SWD: ({pin: {timer, channel, ccer_enabled, moe, gpio_nibble, af_push_pull}}, afio_mapr)."""
    return hw.read_pin_config(OPENOCD_ARGS)


def start_master(armed=False, motors=None, servos=None):
    """Opens the FTDI port and starts the 50 Hz AUX frame stream (hw.FrameStreamer, started).

    The streamer also parses the board's HEARTBEAT (.board_armed, .heartbeat_time) and STATUSTEXT (.status_texts).
    Stop it with .stop(); it also holds .fd, .paused (link loss) and the shared .armed/.motors/.servos.
    """
    file_descriptor = serial_io.open_port(FTDI_PORT, MASTER_BAUD)
    streamer = hw.FrameStreamer(file_descriptor)
    streamer.armed = armed
    if motors is not None:
        streamer.motors = list(motors)
    if servos is not None:
        streamer.servos = list(servos)
    streamer.start()
    return streamer


def wait_until(condition, timeout_s, poll_s=0.05):
    """Polls condition() until it is truthy; returns its value, or None on timeout."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = condition()
        if value:
            return value
        time.sleep(poll_s)
    return None


def preflight():
    """Checks the bench before a run. Returns (ok, problems) with problems a list of texts."""
    problems = []
    if not os.path.exists(FTDI_PORT):
        problems.append("FTDI port missing: " + FTDI_PORT)
    if not stlink_present():
        problems.append("ST-Link not found on USB (ids %s)" % ", ".join(STLINK_USB_IDS))
    if not os.path.exists(ELF_PATH):
        problems.append("firmware ELF missing: " + ELF_PATH)
    if problems:
        return False, problems

    if not hw.verify_firmware(OPENOCD_ARGS, ELF_PATH):
        problems.append("flash content differs from " + ELF_PATH)
        return False, problems

    # Wrong wiring: with the FTDI silent the frame counter must stand still, else a master (EVIS) still feeds it.
    frames_before = read_diag_dict()["auxFrames"]
    time.sleep(SILENCE_PROBE_S)
    frames_after = read_diag_dict()["auxFrames"]
    if frames_after != frames_before:
        problems.append("auxFrames rises while the FTDI is silent: wrong wiring, EVIS still connected to the satellite")
    return not problems, problems
