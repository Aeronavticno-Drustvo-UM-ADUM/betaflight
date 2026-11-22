#!/usr/bin/env python3
"""Hardware output check of the SATELLITEBOARD: servos, motors disarmed and (optional) armed.

  test/tools/hw_output_check.py --port <by-id UART1 adapter> [--armed --props-off]

The script streams ADUM_AUX_CHANNELS at 50 Hz (like the master) and reads the timer compare registers
(CCR) of the running board over ST-Link/OpenOCD. CCR / timer clock = the servo pulse width, so no scope is
needed. Motors use DShot300 (CCR holds bit widths), so motors are checked through the commanded values
(motor[] in us and dmaMotors[i].protocolControl.value) plus static TIM1/DMA checks (step 0, B2). Pin-level
motor signal checks need the logic analyser. Needs the DShot firmware flashed (PWM firmware fails step 0).

Steps:
  0. static DShot config: TIM1 PSC 11, ARR 19, CC1-3 enabled, MOE; DMA1 Ch2/Ch3/Ch6 configured.
  1. servos, disarmed: 6 distinct pulses; the script finds which servo index drives which pin and checks
     the widths against the commanded ones (tolerance --tol-us).
  2. motors, disarmed: motor values in the frames must be ignored, motor[] stays 1000 us, DShot value 0.
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
import struct
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

# Output pins in board order: name -> (timer, channel). Servo 1-6 and motor 1-3 are matched by measurement.
SERVO_PINS = {"PA6": ("TIM3", 1), "PB1": ("TIM3", 4), "PA7": ("TIM3", 2),
              "PB3": ("TIM2", 2), "PB0": ("TIM3", 3), "PA15": ("TIM2", 1)}
MOTOR_PINS = {"PA8": ("TIM1", 1), "PA9": ("TIM1", 2), "PA10": ("TIM1", 3)}

REG_CCER = 0x20            # bit 4*(channel-1) = CCnE (channel output enable)
BDTR_MOE_BIT = 0x8000
GPIO_BASE = {"A": 0x40010800, "B": 0x40010C00}
REG_GPIO_CRL = 0x00        # pins 0-7, 4 bits per pin
REG_GPIO_CRH = 0x04        # pins 8-15
AFIO_MAPR = 0x40010004     # TIM2 remap bits [9:8], SWJ_CFG bits [26:24] (SWJ_CFG may read 0)

NM_PATH = "arm-none-eabi-nm"   # set from --nm in main()
ELF_PATH = ""                  # set from --elf in main()
MOTOR_MIN_US = 1000        # min_command in servo_config_board.txt
SERVO_COUNT = 6
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


def read_pulses_us(openocd_args):
    """Read PSC and CCR of all used timers in one OpenOCD run. Returns {servo pin: pulse in us}, tim1_moe.

    Servos only: the motors use DShot, so TIM1 CCR1-3 hold changing bit widths (see read_motor_commands).
    The core keeps running (no halt needed).
    """
    commands = []
    for timer, base in TIMER_BASE.items():
        commands += ["-c", "mdw 0x%08X 1" % (base + REG_PSC)]
        for channel in range(1, 5):
            commands += ["-c", "mdw 0x%08X 1" % (base + REG_CCR[channel])]
    commands += ["-c", "mdw 0x%08X 1" % (TIMER_BASE["TIM1"] + REG_BDTR)]
    cmd = ["openocd"] + openocd_args + ["-c", "init"] + commands + ["-c", "shutdown"]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    text = result.stdout + result.stderr
    values = {}
    for address, value in re.findall(r"0x([0-9a-fA-F]{8}): ([0-9a-fA-F]{8})", text):
        values[int(address, 16)] = int(value, 16)
    expected_reads = 3 * 5 + 1      # 3 timers x (PSC + 4 CCR) + BDTR
    if len(values) < expected_reads:
        raise SystemExit("OpenOCD read failed (%d of %d registers). Is the ST-Link connected?\n%s" % (
            len(values), expected_reads, text[-800:]))
    pulses = {}
    for pin, (timer, channel) in SERVO_PINS.items():
        base = TIMER_BASE[timer]
        prescaler = values[base + REG_PSC] & 0xFFFF
        ticks = values[base + REG_CCR[channel]] & 0xFFFF
        pulses[pin] = ticks * (prescaler + 1) * 1e6 / TIMER_CLOCK_HZ
    tim1_moe = bool(values[TIMER_BASE["TIM1"] + REG_BDTR] & 0x8000)
    return pulses, tim1_moe


# ---- Motors with DShot: commanded values and static timer/DMA checks (docs/dshot_output.md B2) ----
MAX_MOTORS = 8             # MAX_SUPPORTED_MOTORS in the firmware
MOTOR_COUNT_USED = len(MOTOR_PINS)
DSHOT_VALUE_TOLERANCE = 2
DSHOT_MIN_THROTTLE = 48
DSHOT_MAX_THROTTLE = 2047
DSHOT_STOP_US = 1000       # 1000 us = motor stop = DShot value 0
DSHOT_VALUE_OFFSET = 0     # protocolControl is the first member of motorDmaOutput_t and value its first field (uint16)

TIM1_EXPECTED_PSC = 11
TIM1_EXPECTED_ARR = 19
REG_ARR = 0x2C
CCER_CC1E_CC2E_CC3E = 0x0111       # CC1E bit 0, CC2E bit 4, CC3E bit 8 (CC4E not needed)
DMA1_BASE = 0x40020000
DMA_CHANNEL_STRIDE = 0x14
DMA_REG_CCR = 0x08         # channel 1 CCR; CPAR is +8 from a channel's CCR
DMA_CCR_EN = 0x1
DMA_CCR_DIR = 0x10         # 1 = memory -> peripheral
DMA_CCR_PSIZE_MASK = 0x300
DMA_CCR_MSIZE_MASK = 0xC00
DMA_PSIZE_WORD = 0x200     # 10b = 32 bit
DMA_MSIZE_WORD = 0x800
# DMA1 channel -> TIM1 capture/compare register it must feed (TIM1_CH1/2/3 request lines)
DSHOT_DMA_CHANNELS = {2: 1, 3: 2, 6: 3}


def find_symbol_with_size(nm_path, elf_path, symbol):
    """(address, size in bytes) of a data symbol in the ELF, from 'nm -S'."""
    output = subprocess.run([nm_path, "-S", elf_path], capture_output=True, text=True, check=True).stdout
    for line in output.splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[3].split(".")[0] == symbol:
            return int(fields[0], 16), int(fields[1], 16)
    raise SystemExit("data symbol %s not found in %s" % (symbol, elf_path))


def read_memory_words(openocd_args, addresses):
    """Reads one 32-bit word per address in one OpenOCD run, without halting the core. Returns {address: word}."""
    commands = []
    for address in addresses:
        commands += ["-c", "mdw 0x%08X 1" % address]
    cmd = ["openocd"] + openocd_args + ["-c", "init"] + commands + ["-c", "shutdown"]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    text = result.stdout + result.stderr
    words = {}
    for address_text, word_text in re.findall(r"0x([0-9a-fA-F]{8}): ([0-9a-fA-F]{8})", text):
        words[int(address_text, 16)] = int(word_text, 16)
    missing = [address for address in addresses if address not in words]
    if missing:
        raise SystemExit("OpenOCD read failed (%d of %d words). Is the ST-Link connected?\n%s" % (
            len(addresses) - len(missing), len(addresses), text[-800:]))
    return words


def expected_dshot_value(motor_us):
    """DShot value for a motor command in us: 1000 (stop) -> 0, 1001..2000 -> 48..2047 linear."""
    if motor_us <= DSHOT_STOP_US:
        return 0
    span = DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE
    value = DSHOT_MIN_THROTTLE + (motor_us - (DSHOT_STOP_US + 1)) * span / 999.0
    return min(int(round(value)), DSHOT_MAX_THROTTLE)


def motor_symbol_layout(nm_path, elf_path):
    """Addresses to read the commanded motor values: (motor[] address, dmaMotors address, dmaMotors element stride).

    The stride is sizeof(dmaMotors) / MAX_MOTORS (from the ELF symbol size), so struct size changes are picked up.
    The offset of value inside an element is DSHOT_VALUE_OFFSET (first member; the one assumption, no gdb on host).
    """
    motor_address, _ = find_symbol_with_size(nm_path, elf_path, "motor")
    dma_address, dma_size = find_symbol_with_size(nm_path, elf_path, "dmaMotors")
    return motor_address, dma_address, dma_size // MAX_MOTORS


def read_motor_commands(openocd_args, nm_path, elf_path):
    """Commanded motor values of M1-M3 over SWD (no halt): [{'motor_us': float, 'dshot': int}, ...].

    motor_us is the global motor[] float (microseconds), dshot is dmaMotors[i].protocolControl.value.
    """
    motor_address, dma_address, stride = motor_symbol_layout(nm_path, elf_path)
    addresses = []
    for index in range(MOTOR_COUNT_USED):
        addresses.append(motor_address + 4 * index)
        value_address = dma_address + stride * index + DSHOT_VALUE_OFFSET
        addresses.append(value_address & ~3)
    words = read_memory_words(openocd_args, addresses)
    commands = []
    for index in range(MOTOR_COUNT_USED):
        motor_us = struct.unpack("<f", struct.pack("<I", words[motor_address + 4 * index]))[0]
        value_address = dma_address + stride * index + DSHOT_VALUE_OFFSET
        dshot_word = words[value_address & ~3]
        dshot_value = (dshot_word >> (8 * (value_address & 3))) & 0xFFFF
        commands.append({"motor_us": motor_us, "dshot": dshot_value})
    return commands


def dshot_value_error(command):
    """Difference between the read DShot value and the one expected for the read motor[] value."""
    return command["dshot"] - expected_dshot_value(command["motor_us"])


DSHOT_BUFFER_SYMBOL = "dshotDmaBuffer"
DSHOT_BUFFER_WORDS_PER_MOTOR = 18       # 16 bit widths + 2 reset words (uint32 each)
DSHOT_BIT_COUNT = 16
DSHOT_BIT_0_TICKS = 7
DSHOT_BIT_1_TICKS = 14
DSHOT_FRAME_TOLERANCE = 2               # allowed value difference to the expected one
DSHOT_FRAME_READ_ATTEMPTS = 5           # re-reads when a buffer is caught mid-rewrite


def decode_dshot_buffer(words):
    """Decodes one motor buffer (18 timer-tick words) into a frame dict.

    Keys: bits (str, '?' for an unknown width), value, telemetry, checksum_ok, tail_ok, raw (the words).
    value/telemetry/checksum_ok are None when a bit width is not 7 or 14 (undecodable).
    """
    bit_chars = []
    for width in words[:DSHOT_BIT_COUNT]:
        if width == DSHOT_BIT_1_TICKS:
            bit_chars.append("1")
        elif width == DSHOT_BIT_0_TICKS:
            bit_chars.append("0")
        else:
            bit_chars.append("?")
    bits = "".join(bit_chars)
    tail_ok = all(word == 0 for word in words[DSHOT_BIT_COUNT:])
    frame = {"bits": bits, "value": None, "telemetry": None, "checksum_ok": None,
             "tail_ok": tail_ok, "raw": list(words)}
    if "?" in bits:
        return frame
    packet = int(bits, 2)
    data = packet >> 4
    checksum = packet & 0xF
    expected_checksum = (data ^ (data >> 4) ^ (data >> 8)) & 0xF
    frame["value"] = packet >> 5
    frame["telemetry"] = (packet >> 4) & 1
    frame["checksum_ok"] = checksum == expected_checksum
    return frame


def read_dshot_frames(openocd_args, nm_path, elf_path, attempts=DSHOT_FRAME_READ_ATTEMPTS, retry_wait_s=0.003):
    """DShot frames of M1-M3 decoded from dshotDmaBuffer over SWD (no halt): [frame dict, ...].

    The CPU rewrites the buffer every 1 ms, so a read can mix old and new words. If any motor is
    undecodable or has a bad checksum, all motors are re-read (up to attempts); the last read is returned.
    """
    base_address, _ = find_symbol_with_size(nm_path, elf_path, DSHOT_BUFFER_SYMBOL)
    bytes_per_motor = 4 * DSHOT_BUFFER_WORDS_PER_MOTOR
    frames = []
    for _ in range(attempts):
        addresses = []
        for motor_index in range(MOTOR_COUNT_USED):
            motor_address = base_address + bytes_per_motor * motor_index
            for word_index in range(DSHOT_BUFFER_WORDS_PER_MOTOR):
                addresses.append(motor_address + 4 * word_index)
        words = read_memory_words(openocd_args, addresses)
        frames = []
        for motor_index in range(MOTOR_COUNT_USED):
            motor_words = [words[address] for address in addresses[
                motor_index * DSHOT_BUFFER_WORDS_PER_MOTOR:(motor_index + 1) * DSHOT_BUFFER_WORDS_PER_MOTOR]]
            frames.append(decode_dshot_buffer(motor_words))
        all_good = all(frame["checksum_ok"] for frame in frames)
        if all_good:
            break
        time.sleep(retry_wait_s)
    return frames


def dshot_frame_error(frame, commanded_us):
    """None if the frame is a valid frame for the commanded motor us, else a reason string."""
    if frame["value"] is None:
        return "undecodable frame (bits=%s)" % frame["bits"]
    if not frame["checksum_ok"]:
        return "bad checksum (bits=%s)" % frame["bits"]
    if frame["telemetry"]:
        return "telemetry bit set (bits=%s)" % frame["bits"]
    if not frame["tail_ok"]:
        return "tail words not zero (%s)" % frame["raw"][DSHOT_BIT_COUNT:]
    expected_value = expected_dshot_value(commanded_us)
    if abs(frame["value"] - expected_value) > DSHOT_FRAME_TOLERANCE:
        return "value %d, expected %d +-%d" % (frame["value"], expected_value, DSHOT_FRAME_TOLERANCE)
    return None


def dshot_frame_selftest():
    """Offline decoder check: stop frame OK, 1200 us frame OK, a corrupted word is rejected. Raises on failure."""
    stop_words = [DSHOT_BIT_0_TICKS] * DSHOT_BIT_COUNT + [0, 0]
    stop_frame = decode_dshot_buffer(stop_words)
    assert dshot_frame_error(stop_frame, 1000) is None, stop_frame
    bits_1200 = "0011011111001000"
    words_1200 = [DSHOT_BIT_1_TICKS if bit == "1" else DSHOT_BIT_0_TICKS for bit in bits_1200] + [0, 0]
    frame_1200 = decode_dshot_buffer(words_1200)
    assert frame_1200["value"] == 446 and frame_1200["checksum_ok"], frame_1200
    assert dshot_frame_error(frame_1200, 1200) is None, frame_1200
    words_1200[3] = 9
    assert dshot_frame_error(decode_dshot_buffer(words_1200), 1200) is not None
    return True


def read_dshot_static_config(openocd_args):
    """B2 static checks over SWD (no ESC needed). Returns [(name, ok, detail)].

    TIM1 PSC/ARR/CCER/BDTR and DMA1 Ch2/Ch3/Ch6 (CPAR = TIM1 CCR1/2/3, memory -> peripheral, 32-bit sizes).
    The DMA EN bit is only reported: a non-circular transfer clears it after each burst.
    """
    tim1 = TIMER_BASE["TIM1"]
    addresses = [tim1 + REG_PSC, tim1 + REG_ARR, tim1 + REG_CCER, tim1 + REG_BDTR]
    for channel in DSHOT_DMA_CHANNELS:
        channel_base = DMA1_BASE + DMA_REG_CCR + DMA_CHANNEL_STRIDE * (channel - 1)
        addresses += [channel_base, channel_base + 8]      # CCR, CPAR
    words = read_memory_words(openocd_args, addresses)

    checks = []
    psc = words[tim1 + REG_PSC] & 0xFFFF
    arr = words[tim1 + REG_ARR] & 0xFFFF
    ccer = words[tim1 + REG_CCER] & 0xFFFF
    bdtr = words[tim1 + REG_BDTR] & 0xFFFF
    checks.append(("TIM1 PSC == %d" % TIM1_EXPECTED_PSC, psc == TIM1_EXPECTED_PSC, "PSC=%d" % psc))
    checks.append(("TIM1 ARR == %d (DShot300 bit period)" % TIM1_EXPECTED_ARR, arr == TIM1_EXPECTED_ARR, "ARR=%d" % arr))
    checks.append(("TIM1 CC1E/CC2E/CC3E set", (ccer & CCER_CC1E_CC2E_CC3E) == CCER_CC1E_CC2E_CC3E,
                   "CCER=0x%04X" % ccer))
    checks.append(("TIM1 BDTR MOE set", bool(bdtr & BDTR_MOE_BIT), "BDTR=0x%04X" % bdtr))
    for channel, timer_channel in DSHOT_DMA_CHANNELS.items():
        channel_base = DMA1_BASE + DMA_REG_CCR + DMA_CHANNEL_STRIDE * (channel - 1)
        dma_ccr = words[channel_base]
        dma_cpar = words[channel_base + 8]
        wanted_cpar = tim1 + REG_CCR[timer_channel]
        configured = (dma_cpar == wanted_cpar
                      and bool(dma_ccr & DMA_CCR_DIR)
                      and (dma_ccr & DMA_CCR_PSIZE_MASK) == DMA_PSIZE_WORD
                      and (dma_ccr & DMA_CCR_MSIZE_MASK) == DMA_MSIZE_WORD)
        checks.append(("DMA1 Ch%d configured for TIM1 CCR%d" % (channel, timer_channel), configured,
                       "CCR=0x%08X CPAR=0x%08X (want 0x%08X), EN=%d" % (
                           dma_ccr, dma_cpar, wanted_cpar, dma_ccr & DMA_CCR_EN)))
    return checks


def gpio_config_register(pin_name):
    """Address of the GPIO CRL/CRH register and the bit shift of the pin's nibble, from a name like 'PA15'."""
    port_letter = pin_name[1]
    pin_number = int(pin_name[2:])
    register = REG_GPIO_CRL if pin_number < 8 else REG_GPIO_CRH
    shift = 4 * (pin_number % 8)
    return GPIO_BASE[port_letter] + register, shift


def read_pin_config(openocd_args):
    """Read timer output enables and GPIO modes of all used pins in one OpenOCD run.

    Returns ({pin: {timer, channel, ccer_enabled, moe, gpio_nibble, af_push_pull}}, afio_mapr).
    moe is None for pins not on TIM1. The core keeps running (mdw reads need no halt).
    """
    pins = dict(SERVO_PINS)
    pins.update(MOTOR_PINS)
    addresses = [AFIO_MAPR, TIMER_BASE["TIM1"] + REG_BDTR]
    for timer_base in TIMER_BASE.values():
        addresses.append(timer_base + REG_CCER)
    for pin_name in pins:
        register_address, _shift = gpio_config_register(pin_name)
        addresses.append(register_address)
    addresses = sorted(set(addresses))

    commands = []
    for address in addresses:
        commands += ["-c", "mdw 0x%08X 1" % address]
    cmd = ["openocd"] + openocd_args + ["-c", "init"] + commands + ["-c", "shutdown"]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    text = result.stdout + result.stderr
    values = {}
    for address, value in re.findall(r"0x([0-9a-fA-F]{8}): ([0-9a-fA-F]{8})", text):
        values[int(address, 16)] = int(value, 16)
    if len(values) < len(addresses):
        raise SystemExit("OpenOCD pin config read failed (%d of %d registers). Is the ST-Link connected?\n%s" % (
            len(values), len(addresses), text[-800:]))

    config = {}
    for pin_name, (timer, channel) in pins.items():
        ccer = values[TIMER_BASE[timer] + REG_CCER]
        channel_enable_bit = 1 << (4 * (channel - 1))
        moe = None
        if timer == "TIM1":
            moe = bool(values[TIMER_BASE["TIM1"] + REG_BDTR] & BDTR_MOE_BIT)
        register_address, shift = gpio_config_register(pin_name)
        nibble = (values[register_address] >> shift) & 0xF
        mode_is_output = (nibble & 0x3) != 0
        cnf_is_alternate_push_pull = (nibble & 0xC) == 0x8
        config[pin_name] = {
            "timer": timer,
            "channel": channel,
            "ccer_enabled": bool(ccer & channel_enable_bit),
            "moe": moe,
            "gpio_nibble": nibble,
            "af_push_pull": mode_is_output and cnf_is_alternate_push_pull,
        }
    return config, values[AFIO_MAPR]


def pin_config_problems(config):
    """Human-readable problems for a read_pin_config() result (a (config, afio_mapr) tuple or the dict). Empty = OK."""
    if isinstance(config, tuple):
        config = config[0]
    problems = []
    for pin_name, pin in config.items():
        label = "%s (%s CH%d)" % (pin_name, pin["timer"], pin["channel"])
        if not pin["ccer_enabled"]:
            problems.append("%s: timer channel output not enabled (CCER CC%dE = 0)" % (label, pin["channel"]))
        if pin["moe"] is False:
            problems.append("%s: TIM1 main output not enabled (BDTR MOE = 0)" % label)
        if not pin["af_push_pull"]:
            problems.append("%s: GPIO not alternate-function push-pull output (nibble 0x%X)" % (label, pin["gpio_nibble"]))
    return problems


def settle_and_read(openocd_args, wait_s=0.4):
    """Let the board process a few frames, then read the pulses."""
    time.sleep(wait_s)
    return read_pulses_us(openocd_args)


def settle_and_read_motors(openocd_args, wait_s=0.4):
    """Let the board process a few frames, then read the commanded motor values (NM_PATH/ELF_PATH set in main)."""
    time.sleep(wait_s)
    return read_motor_commands(openocd_args, NM_PATH, ELF_PATH)


def check_motor_values(results, label, commands, wanted_us_list, tol_us):
    """Checks motor[i] (us) and the DShot value for each motor against the wanted us (index order M1-M3)."""
    for index, command in enumerate(commands):
        wanted_us = wanted_us_list[index]
        us_error = abs(command["motor_us"] - wanted_us)
        dshot_error = command["dshot"] - expected_dshot_value(wanted_us)
        results.check("%s M%d motor[] within %.0f us of %.0f" % (label, index + 1, tol_us, wanted_us),
                      us_error <= tol_us, "motor[]=%.1f us" % command["motor_us"])
        results.check("%s M%d DShot value within +-%d of %d" % (
            label, index + 1, DSHOT_VALUE_TOLERANCE, expected_dshot_value(wanted_us)),
            abs(dshot_error) <= DSHOT_VALUE_TOLERANCE, "value=%d" % command["dshot"])


def check_motor_frames(results, label, frames, wanted_us_list):
    """Checks the decoded DShot frame of each motor against the wanted us (index order M1-M3)."""
    for index, frame in enumerate(frames):
        error = dshot_frame_error(frame, wanted_us_list[index])
        results.check("%s M%d DShot frame valid (checksum, tail, value)" % (label, index + 1),
                      error is None, error or "bits=%s value=%s" % (frame["bits"], frame["value"]))


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


def check_servos(streamer, openocd_args, tol_us, results):
    """Step 1: distinct pulses on servos 1-6, disarmed. Finds and checks the index -> pin mapping."""
    print("Step 1: servos, disarmed")
    streamer.armed = False
    first_set = [1100, 1250, 1400, 1550, 1700, 1900]
    second_set = list(reversed(first_set))
    mapping_first = None
    for label, values in (("set A", first_set), ("set B", second_set)):
        streamer.servos[:SERVO_COUNT] = values
        pulses, moe = settle_and_read(openocd_args)
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


def check_motors_disarmed(streamer, openocd_args, tol_us, results):
    """Step 2: motor values in disarmed frames must not reach the outputs."""
    print("Step 2: motors, disarmed")
    streamer.armed = False
    streamer.motors = [1300, 1400, 1500]
    commands = settle_and_read_motors(openocd_args)
    check_motor_values(results, "disarmed", commands, [MOTOR_MIN_US] * MOTOR_COUNT_USED, tol_us)
    frames = read_dshot_frames(openocd_args, NM_PATH, ELF_PATH)
    check_motor_frames(results, "disarmed", frames, [MOTOR_MIN_US] * MOTOR_COUNT_USED)
    streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT


def check_motors_armed(streamer, openocd_args, diag_address, tol_us, results):
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
    commands = settle_and_read_motors(openocd_args)
    check_motor_values(results, "armed", commands, commanded, tol_us)
    disarm_time = time.monotonic()
    streamer.armed = False
    streamer.motors = [1300, 1300, 1300]
    time.sleep(SWD_SETTLE_S)
    disarm_state = read_board_state(openocd_args, diag_address)
    swd_check(results, "board DISARMED after a disarmed frame (SWD)", disarm_state,
              lambda state: state.last_to_state == STATE_DISARMED)
    results.check("HEARTBEAT reports DISARMED after a disarmed frame",
                  streamer.wait_board_state(False, 3.0, since=disarm_time) is not None, describe_state(disarm_state))
    commands = settle_and_read_motors(openocd_args)
    check_motor_values(results, "after disarm", commands, [MOTOR_MIN_US] * MOTOR_COUNT_USED, tol_us)
    streamer.motors = [mav.PULSE_IGNORE] * MOTOR_COUNT
    return True


def check_failsafe(streamer, openocd_args, diag_address, tol_us, results):
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
    commands = settle_and_read_motors(openocd_args, wait_s=1.2)
    check_motor_values(results, "1.2 s after the last frame", commands, [MOTOR_MIN_US] * MOTOR_COUNT_USED, tol_us)
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
    global NM_PATH, ELF_PATH
    NM_PATH = args.nm
    ELF_PATH = args.elf
    diag_address = find_symbol_address(args.nm, args.elf, "servoDriverDiag")
    fd = serial_io.open_port(args.port, args.baud)
    streamer = FrameStreamer(fd)
    streamer.start()
    results = Results()
    try:
        if streamer.wait_board_state(False, 5.0) is None:
            raise SystemExit("no HEARTBEAT from the board in 5 s (wrong port, or it is armed): nothing tested")
        print("Board answers, disarmed.")
        print("Step 0: static DShot output config (B2)")
        for name, ok, detail in read_dshot_static_config(openocd_args):
            results.check(name, ok, detail)
        check_servos(streamer, openocd_args, args.tol_us, results)
        check_motors_disarmed(streamer, openocd_args, args.tol_us, results)
        if args.armed:
            if check_motors_armed(streamer, openocd_args, diag_address, args.tol_us, results):
                check_failsafe(streamer, openocd_args, diag_address, args.tol_us, results)
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
