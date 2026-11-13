#!/usr/bin/env python3
"""Hardware output check of the SATELLITEBOARD: servos, motors disarmed and (optional) armed.

  test/tools/hw_output_check.py --port <by-id UART1 adapter> [--armed --props-off]

The script streams ADUM_AUX_CHANNELS at 50 Hz (like the master) and reads the timer compare registers
(CCR) of the running board over ST-Link/OpenOCD. CCR / timer clock = the pulse width the pin outputs, so
no scope is needed. Scope stays useful for 50 Hz / 200 Hz timing only.

Steps:
  1. servos, disarmed: 5 distinct pulses; the script finds which servo index drives which pin and checks
     the widths against the commanded ones (tolerance --tol-us).
  2. motors, disarmed: motor values in the frames must be ignored, outputs stay at min_command.
  3. (--armed) motors, armed: arm edge, board ARMED (SWD diag) and HEARTBEAT shows ARMED, motors follow the
     frame; then disarm.
  4. (--armed) link loss while armed: frames stop, motors go to min_command (failsafe, SWD state FAILSAFE).
  The arm state is read from servoDriverDiag over SWD, so a slow UART adapter delaying HEARTBEAT cannot fake
  an arming failure: the SWD check tells the board's truth, the HEARTBEAT check the reporting path.
  Steps 3 and 4 spin the motors at a low pulse. Propellers off, ESCs/motors may be disconnected.

Exit code 0 = all checks passed. Standard library only. The ST-Link must be connected.
"""
import argparse
import os
import re
import subprocess
import sys
import threading
import time
from collections import namedtuple

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)
sys.path.insert(0, os.path.join(TOOLS_DIR, "..", "common"))
import mavlink_gen as mav  # noqa: E402
import send_aux_channels as serial_io  # noqa: E402  (open_port, write_all, read_available)
import read_diag  # noqa: E402  (FIELDS, read_words)

TIMER_CLOCK_HZ = 72_000_000
TIMER_BASE = {"TIM1": 0x40012C00, "TIM2": 0x40000000, "TIM3": 0x40000400}
REG_PSC = 0x28
REG_CCR = {1: 0x34, 2: 0x38, 3: 0x3C, 4: 0x40}
REG_BDTR = 0x44            # TIM1 only, bit 15 = MOE (main output enable)

# Output pins in board order: name -> (timer, channel). Servo 1-5 and motor 1-3 are matched by measurement.
SERVO_PINS = {"PA8": ("TIM1", 1), "PA9": ("TIM1", 2), "PA10": ("TIM1", 3), "PA15": ("TIM2", 1), "PB3": ("TIM2", 2)}
MOTOR_PINS = {"PA6": ("TIM3", 1), "PB0": ("TIM3", 3), "PB1": ("TIM3", 4)}

MOTOR_MIN_US = 1000        # min_command in servo_config_board.txt
ONESHOT125_DIVIDER = 8     # OneShot125 pulse = command / 8 (1000 us -> 125 us)
SERVO_COUNT = 5
ARM_EXPECTED_S = 1.5       # arm edge -> HEARTBEAT shows ARMED (HEARTBEAT is sent at 1 Hz)
ARM_WAIT_MAX_S = 15.0      # how long to wait for a late arm before giving up
MOTOR_COUNT = 3
SWD_SETTLE_S = 0.3         # command -> SWD read of the board state (the board needs a few 20 ms frames)

DIAG_MAGIC = 0x5D1A6001
STATE_DISARMED, STATE_ARMED, STATE_FAILSAFE = 0, 1, 2
STATE_NAMES = {STATE_DISARMED: "DISARMED", STATE_ARMED: "ARMED", STATE_FAILSAFE: "FAILSAFE"}

BoardState = namedtuple("BoardState", "last_to_state armed_at_frame first_armed_frame edges_not_ready last_not_ready_reason")


class Results:
    """Collects check lines and the failure count."""

    def __init__(self):
        self.failed = 0

    def check(self, name, ok, detail=""):
        print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
        if not ok:
            self.failed += 1


class FrameStreamer(threading.Thread):
    """Sends ADUM_AUX_CHANNELS at 50 Hz from the shared state and tracks the board's HEARTBEAT."""

    def __init__(self, fd):
        super().__init__(daemon=True)
        self.fd = fd
        self.generator = mav.MavlinkGenerator()
        self.parser = mav.FrameParser()
        self.armed = False
        self.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT
        self.servos = [1500] * 8
        self.paused = False            # True = send nothing (link loss)
        self.board_armed = None        # last HEARTBEAT arm state, None until the first one
        self.heartbeat_time = 0.0
        self.running = True
        self.status_texts = []         # STATUSTEXT strings received from the board

    def run(self):
        next_frame = time.monotonic()
        while self.running:
            now = time.monotonic()
            if now >= next_frame:
                if not self.paused:
                    frame = self.generator.aux_channels(self.armed, list(self.motors), list(self.servos))
                    serial_io.write_all(self.fd, frame)
                next_frame += 0.02
                if next_frame < now - 1.0:
                    next_frame = now
            for msgid, payload in self.parser.feed(serial_io.read_available(self.fd)):
                if msgid == mav.MSG_HEARTBEAT:
                    self.board_armed = mav.decode_heartbeat(payload)["armed"]
                    self.heartbeat_time = time.monotonic()
                elif msgid == mav.MSG_STATUSTEXT:
                    self.status_texts.append(str(mav.decode_statustext(payload)))
            time.sleep(0.001)

    def stop(self):
        self.running = False
        self.join(timeout=1.0)

    def wait_board_state(self, armed, timeout_s, since=None):
        """Wait for a HEARTBEAT, received after `since` (default: now), with the given arm state. Returns the wait in s or None."""
        start = time.monotonic() if since is None else since
        while time.monotonic() - start < timeout_s:
            if self.board_armed is armed and self.heartbeat_time > start:
                return time.monotonic() - start
            time.sleep(0.02)
        return None


def read_board_state(openocd_args, diag_address):
    """Read the arm state from servoDriverDiag over SWD (no halt). None when the firmware has no diag block."""
    words = read_diag.read_words(diag_address, len(read_diag.FIELDS), " ".join(openocd_args))
    values = dict(zip(read_diag.FIELDS, words))
    if values["magic"] != DIAG_MAGIC:
        return None
    return BoardState(values["lastToState"], values["armedAtFrameNo"], values["firstArmedFrameNo"],
                      values["edgesNotReady"], values["lastNotReadyReason"])


def describe_state(state):
    """Short text of a BoardState for check detail lines."""
    if state is None:
        return "SWD n/a"
    return "SWD %s, armedAt %d, firstArmed %d, edgesNotReady %d (reason %d)" % (
        STATE_NAMES.get(state.last_to_state, "?"), state.armed_at_frame, state.first_armed_frame,
        state.edges_not_ready, state.last_not_ready_reason)


def swd_check(results, name, state, ok_condition):
    """Add an SWD check; with no diag in the firmware only print a note (HEARTBEAT checks still run)."""
    if state is None:
        print("    (firmware has no diag block: SWD check '%s' skipped, HEARTBEAT only)" % name)
        return
    results.check(name, ok_condition(state), describe_state(state))


def find_symbol_address(nm_path, elf_path, symbol):
    """Address of a function in the firmware ELF (the name may carry a .lto_priv suffix)."""
    output = subprocess.run([nm_path, elf_path], capture_output=True, text=True, check=True).stdout
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2].split(".")[0] == symbol:
            return int(fields[0], 16)
    raise SystemExit("symbol %s not found in %s" % (symbol, elf_path))


def verify_firmware(openocd_args, elf_path):
    """True when the flash content equals the ELF (so the symbol addresses are valid)."""
    cmd = ["openocd"] + openocd_args + ["-c", "init", "-c", "verify_image %s" % elf_path, "-c", "resume", "-c", "shutdown"]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return "verified" in (result.stdout + result.stderr)


def read_pulses_us(openocd_args, sample_address):
    """Read PSC and CCR of all used timers in one OpenOCD run. Returns {pin: pulse in us}, tim1_moe.

    OneShot motors have CCR = 0 except between the write and the end of the update, so the core is halted
    on a breakpoint at pwmCompleteOneshotMotorUpdate (CCRs are valid there) and the registers are read.
    """
    commands = []
    for timer, base in TIMER_BASE.items():
        commands += ["-c", "mdw 0x%08X 1" % (base + REG_PSC)]
        for channel in range(1, 5):
            commands += ["-c", "mdw 0x%08X 1" % (base + REG_CCR[channel])]
    commands += ["-c", "mdw 0x%08X 1" % (TIMER_BASE["TIM1"] + REG_BDTR)]
    halt_setup = ["-c", "halt", "-c", "bp 0x%08X 2 hw" % sample_address, "-c", "resume", "-c", "wait_halt 3000"]
    halt_release = ["-c", "rbp 0x%08X" % sample_address, "-c", "resume"]     # the board runs on again
    cmd = ["openocd"] + openocd_args + ["-c", "init"] + halt_setup + commands + halt_release + ["-c", "shutdown"]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    text = result.stdout + result.stderr
    values = {}
    for address, value in re.findall(r"0x([0-9a-fA-F]{8}): ([0-9a-fA-F]{8})", text):
        values[int(address, 16)] = int(value, 16)
    expected_reads = 3 * 5 + 1
    if len(values) < expected_reads:
        raise SystemExit("OpenOCD read failed (%d of %d registers). Is the ST-Link connected?\n%s" % (
            len(values), expected_reads, text[-800:]))
    pulses = {}
    for pin, (timer, channel) in list(SERVO_PINS.items()) + list(MOTOR_PINS.items()):
        base = TIMER_BASE[timer]
        prescaler = values[base + REG_PSC] & 0xFFFF
        ticks = values[base + REG_CCR[channel]] & 0xFFFF
        pulses[pin] = ticks * (prescaler + 1) * 1e6 / TIMER_CLOCK_HZ
    tim1_moe = bool(values[TIMER_BASE["TIM1"] + REG_BDTR] & 0x8000)
    return pulses, tim1_moe


def settle_and_read(openocd_args, sample_address, wait_s=0.4):
    """Let the board process a few frames, then read the pulses."""
    time.sleep(wait_s)
    return read_pulses_us(openocd_args, sample_address)


def match_outputs(commanded_us, measured_pins, pulses, tol_us):
    """Match each commanded value to the pin with the closest pulse. Returns {index: pin}, worst error in us."""
    mapping = {}
    worst = 0.0
    free_pins = list(measured_pins)
    for index, wanted in enumerate(commanded_us):
        best_pin = min(free_pins, key=lambda pin: abs(pulses[pin] - wanted))
        worst = max(worst, abs(pulses[best_pin] - wanted))
        mapping[index] = best_pin
        free_pins.remove(best_pin)
    return mapping, worst


def check_servos(streamer, openocd_args, sample_address, tol_us, results):
    """Step 1: distinct pulses on servos 1-5, disarmed. Finds and checks the index -> pin mapping."""
    print("Step 1: servos, disarmed")
    streamer.armed = False
    first_set = [1100, 1300, 1500, 1700, 1900]
    second_set = [1900, 1700, 1500, 1300, 1100]
    mapping_first = None
    for label, values in (("set A", first_set), ("set B", second_set)):
        streamer.servos[:SERVO_COUNT] = values
        pulses, moe = settle_and_read(openocd_args, sample_address)
        mapping, worst = match_outputs(values, SERVO_PINS, pulses, tol_us)
        results.check("servos %s: widths within %.0f us" % (label, tol_us), worst <= tol_us,
                      "worst error %.1f us, measured %s" % (worst, {p: round(pulses[p], 1) for p in SERVO_PINS}))
        if mapping_first is None:
            mapping_first = mapping
            print("    servo index -> pin: %s" % {i + 1: pin for i, pin in mapping.items()})
        else:
            results.check("servos: same index -> pin mapping in set B", mapping == mapping_first)
        results.check("TIM1 main output enabled (MOE)", moe)
    streamer.servos[:SERVO_COUNT] = [1500] * SERVO_COUNT
    return mapping_first


def check_motors_disarmed(streamer, openocd_args, sample_address, tol_us, results):
    """Step 2: motor values in disarmed frames must not reach the outputs."""
    print("Step 2: motors, disarmed")
    streamer.armed = False
    streamer.motors = [1300, 1400, 1500]
    pulses, _ = settle_and_read(openocd_args, sample_address)
    idle_us = MOTOR_MIN_US / ONESHOT125_DIVIDER
    for pin in MOTOR_PINS:
        results.check("motor %s stays at min_command (%.1f us)" % (pin, idle_us),
                      abs(pulses[pin] - idle_us) <= tol_us, "measured %.1f us" % pulses[pin])
    streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT


def check_motors_armed(streamer, openocd_args, sample_address, diag_address, tol_us, results):
    """Step 3: arm edge, motors follow the frame, disarm returns them to min_command."""
    print("Step 3: motors, armed (props off!)")
    streamer.armed = False
    streamer.motors = [1000, 1000, 1000]
    time.sleep(1.0)                      # disarmed frames first, so the board sees an arm edge
    arm_time = time.monotonic()
    streamer.armed = True
    commanded = [1100, 1200, 1300]
    streamer.motors = list(commanded)
    time.sleep(SWD_SETTLE_S)
    arm_state = read_board_state(openocd_args, diag_address)         # the board's own view, independent of the UART
    swd_check(results, "board ARMED on the edge frame (SWD)", arm_state,
              lambda state: state.last_to_state == STATE_ARMED and state.armed_at_frame == state.first_armed_frame)
    arm_wait = streamer.wait_board_state(True, ARM_WAIT_MAX_S, since=arm_time)
    results.check("HEARTBEAT reports ARMED within %.1f s (reporting path, HEARTBEAT is 1 Hz)" % ARM_EXPECTED_S,
                  arm_wait is not None and arm_wait <= ARM_EXPECTED_S,
                  "armed after %s; %s" % ("never (%d s)" % ARM_WAIT_MAX_S if arm_wait is None else "%.1f s" % arm_wait,
                                          describe_state(arm_state)))
    if arm_wait is None:
        print("    board STATUSTEXT: %s" % (streamer.status_texts[-3:] or "none"))
        streamer.armed = False
        streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT
        return False
    pulses, _ = settle_and_read(openocd_args, sample_address)
    wanted_us = [value / ONESHOT125_DIVIDER for value in commanded]
    mapping, worst = match_outputs(wanted_us, MOTOR_PINS, pulses, tol_us)
    results.check("armed motor widths within %.0f us" % tol_us, worst <= tol_us,
                  "worst error %.1f us, measured %s" % (worst, {p: round(pulses[p], 1) for p in MOTOR_PINS}))
    print("    motor index -> pin: %s" % {i + 1: pin for i, pin in mapping.items()})
    disarm_time = time.monotonic()
    streamer.armed = False
    streamer.motors = [1300, 1300, 1300]
    time.sleep(SWD_SETTLE_S)
    disarm_state = read_board_state(openocd_args, diag_address)
    swd_check(results, "board DISARMED after a disarmed frame (SWD)", disarm_state,
              lambda state: state.last_to_state == STATE_DISARMED)
    results.check("HEARTBEAT reports DISARMED after a disarmed frame",
                  streamer.wait_board_state(False, 3.0, since=disarm_time) is not None, describe_state(disarm_state))
    pulses, _ = settle_and_read(openocd_args, sample_address)
    idle_us = MOTOR_MIN_US / ONESHOT125_DIVIDER
    worst_idle = max(abs(pulses[pin] - idle_us) for pin in MOTOR_PINS)
    results.check("motors back at min_command after disarm", worst_idle <= tol_us, "worst %.1f us" % worst_idle)
    streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT
    return True


def check_failsafe(streamer, openocd_args, sample_address, diag_address, tol_us, results):
    """Step 4: armed, then no frames: motors go to min_command, HEARTBEAT shows failsafe (not armed flag)."""
    print("Step 4: link loss while armed")
    streamer.armed = False
    streamer.motors = [1000, 1000, 1000]
    time.sleep(1.0)
    streamer.armed = True
    streamer.motors = [1200, 1200, 1200]
    if streamer.wait_board_state(True, ARM_WAIT_MAX_S) is None:
        results.check("board armed before the link loss", False)
        streamer.armed = False
        return
    streamer.paused = True               # master goes silent
    pulses, _ = settle_and_read(openocd_args, sample_address, wait_s=1.2)
    idle_us = MOTOR_MIN_US / ONESHOT125_DIVIDER
    worst_idle = max(abs(pulses[pin] - idle_us) for pin in MOTOR_PINS)
    results.check("motors at min_command 1.2 s after the last frame", worst_idle <= tol_us, "worst %.1f us" % worst_idle)
    failsafe_state = read_board_state(openocd_args, diag_address)
    swd_check(results, "board in FAILSAFE after the link loss while armed (SWD)", failsafe_state,
              lambda state: state.last_to_state == STATE_FAILSAFE)
    streamer.armed = False               # leave the board disarmed: the next frames are disarmed
    streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT
    streamer.paused = False
    results.check("board disarmed after the link returns with a disarmed frame", streamer.wait_board_state(False, 3.0) is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="UART1 adapter, /dev/serial/by-id/...-if06")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--armed", action="store_true", help="also run the armed motor steps 3 and 4")
    parser.add_argument("--props-off", action="store_true", help="confirm propellers are removed (needed with --armed)")
    parser.add_argument("--tol-us", type=float, default=2.0, help="pulse width tolerance in us (default 2)")
    parser.add_argument("--elf", default=os.path.join(TOOLS_DIR, "..", "..", "obj", "main", "betaflight_SATELLITEBOARD.elf"),
                        help="firmware ELF that is flashed on the board")
    parser.add_argument("--nm", default=os.path.join(TOOLS_DIR, "..", "..", "tools", "gcc-arm-none-eabi-9-2020-q2-update",
                                                     "bin", "arm-none-eabi-nm"), help="arm-none-eabi-nm")
    parser.add_argument("--openocd-args", default="-f interface/stlink.cfg -f target/stm32f1x.cfg",
                        help="OpenOCD interface/target options")
    args = parser.parse_args()
    if args.armed and not args.props_off:
        raise SystemExit("--armed spins the motors: remove the propellers and add --props-off")

    openocd_args = args.openocd_args.split()
    if not verify_firmware(openocd_args, args.elf):
        raise SystemExit("flash differs from %s: flash that firmware first (symbol addresses would be wrong)" % args.elf)
    sample_address = find_symbol_address(args.nm, args.elf, "pwmCompleteOneshotMotorUpdate")
    diag_address = find_symbol_address(args.nm, args.elf, "servoDriverDiag")
    fd = serial_io.open_port(args.port, args.baud)
    streamer = FrameStreamer(fd)
    streamer.start()
    results = Results()
    try:
        if streamer.wait_board_state(False, 5.0) is None:
            raise SystemExit("no HEARTBEAT from the board in 5 s (wrong port, or it is armed): nothing tested")
        print("Board answers, disarmed.")
        check_servos(streamer, openocd_args, sample_address, args.tol_us, results)
        check_motors_disarmed(streamer, openocd_args, sample_address, args.tol_us, results)
        if args.armed:
            if check_motors_armed(streamer, openocd_args, sample_address, diag_address, args.tol_us, results):
                check_failsafe(streamer, openocd_args, sample_address, diag_address, args.tol_us, results)
    finally:
        # leave the board disarmed, then silence
        streamer.armed = False
        streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT
        streamer.paused = False
        time.sleep(0.2)
        streamer.stop()
        os.close(fd)
    print("\n%s: %d check(s) failed" % ("FAILED" if results.failed else "PASSED", results.failed))
    return 1 if results.failed else 0


if __name__ == "__main__":
    sys.exit(main())
