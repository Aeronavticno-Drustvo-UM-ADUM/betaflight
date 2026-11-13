#!/usr/bin/env python3
"""Read servoDriverDiag from a running SATELLITEBOARD over OpenOCD (SWD), without halting the core."""

import argparse
import re
import subprocess
import sys

FIELDS = [
    "magic", "resetCsr", "bootCount",
    "auxFrames", "auxArmedFrames", "auxDisarmedFrames", "newSessions",
    "armEdges", "edgeRejects", "lastRejectReason", "rejectTextsSent", "rejectTextsDropped",
    "txDrops", "parseErrors", "stateChanges",
    "lastFromState", "lastToState", "lastCause", "lastAuxFrameNo", "lastTransitionMs",
    "maxRxBytesPerCycle", "maxCycleGapUs", "firstArmedFrameNo", "armedAtFrameNo",
]

CSR_FLAGS = [(26, "PIN"), (27, "POR"), (28, "SFT"), (29, "IWDG"), (30, "WWDG"), (31, "LPWR")]


def find_symbol_address(elf_path):
    """Address of servoDriverDiag from nm."""
    output = subprocess.check_output(["nm", elf_path], text=True)
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == "servoDriverDiag":
            return int(parts[0], 16)
    sys.exit("symbol servoDriverDiag not found in " + elf_path)


def read_words(address, count, openocd_args):
    """One mdw over OpenOCD; returns the words as integers."""
    read_command = "mdw 0x%08x %d" % (address, count)
    # separate -c commands and "shutdown": a combined "init; ...; exit" printed no memory lines
    arguments = ["openocd"] + openocd_args.split() + ["-c", "init", "-c", read_command, "-c", "shutdown"]
    result = subprocess.run(arguments, capture_output=True, text=True)
    words = []
    for line in (result.stdout + result.stderr).splitlines():
        match = re.match(r"^0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]{8}\s*)+)$", line.strip())
        if match:
            words.extend(int(word, 16) for word in match.group(1).split())
    if len(words) < count:
        sys.exit("read %d of %d words; openocd output:\n%s" % (len(words), count, result.stdout + result.stderr))
    return words[:count]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", default="obj/main/betaflight_SATELLITEBOARD.elf")
    parser.add_argument("--openocd-args", default="-f interface/stlink.cfg -f target/stm32f1x.cfg")
    args = parser.parse_args()

    address = find_symbol_address(args.elf)
    words = read_words(address, len(FIELDS), args.openocd_args)
    values = dict(zip(FIELDS, words))

    for name in FIELDS:
        print("%s=%d" % (name, values[name]))
    print("magic_hex=0x%08X%s" % (values["magic"], "" if values["magic"] == 0x5D1A6001 else "  (BAD MAGIC)"))
    csr = values["resetCsr"]
    flags = [name for bit, name in CSR_FLAGS if csr & (1 << bit)]
    print("resetCsr_hex=0x%08X flags=%s" % (csr, ",".join(flags) if flags else "none"))


if __name__ == "__main__":
    main()
