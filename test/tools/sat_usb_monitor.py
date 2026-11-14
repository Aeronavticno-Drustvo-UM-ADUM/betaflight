#!/usr/bin/env python3
"""Live view of the SATELLITEBOARD over its USB port (MSP), no ST-Link needed.

Prints, every period: the satellite state (DISARMED / ARMED / FAILSAFE) and the servo 1-6 and motor 1-3 values the
board applied from the last ADUM_AUX_CHANNELS frame (S1-S6, M1-M3), with their PCB output header (O1-O9).
Servo/motor values need firmware from 2026-11-14 or later (servo_driver.c fills servo[] / motor[] for MSP).

Usage:  python3 test/tools/sat_usb_monitor.py [--port /dev/serial/by-id/...] [--period 0.5]
Close the configurator first: one program on the port at a time.
"""

import argparse
import glob
import struct
import sys
import time

import serial

MSP_STATUS = 101
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_BOXIDS = 119
BOX_ARM = 0                 # permanent box ids (msp_box.c)
BOX_FAILSAFE = 27

SERVO_OUTPUTS = ["O1", "O9", "O2", "O8", "O3", "O7"]  # PCB header of S1-S6 (target.c, docs/adum/sat_pinout.png)
MOTOR_OUTPUTS = ["O4", "O6", "O5"]                    # PCB header of M1-M3


def default_port():
    """The satellite's USB virtual COM port (STM32 VCP), if exactly one is connected."""
    ports = glob.glob("/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_COM_Port_*")
    if len(ports) != 1:
        sys.exit("give --port (found %d STM32 VCP ports)" % len(ports))
    return ports[0]


class Msp:
    """Minimal MSP v1 client: request without payload, return the reply payload."""

    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 115200, timeout=1)

    def request(self, command):
        """Sends one request and returns the payload bytes of its reply."""
        self.port.reset_input_buffer()
        self.port.write(b"$M<" + bytes([0, command, command]))       # size 0, checksum = size ^ command
        header = self.port.read(5)
        if len(header) < 5 or header[:3] != b"$M>" or header[4] != command:
            raise IOError("no valid reply to MSP %d (header %r)" % (command, header))
        payload = self.port.read(header[3])
        self.port.read(1)                                              # checksum, not checked
        return payload


def read_state(msp, box_ids):
    """Satellite state from the ARM and FAILSAFE boxes in MSP_STATUS."""
    status = msp.request(MSP_STATUS)
    flags = struct.unpack_from("<I", status, 6)[0]                     # first 32 active-box bits
    active = {box_ids[bit] for bit in range(min(32, len(box_ids))) if flags & (1 << bit)}
    if BOX_FAILSAFE in active:
        return "FAILSAFE"
    if BOX_ARM in active:
        return "ARMED"
    return "DISARMED"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="satellite USB port (default: the only STM32 VCP)")
    parser.add_argument("--period", type=float, default=0.5, help="seconds between prints")
    args = parser.parse_args()

    msp = Msp(args.port or default_port())
    box_ids = list(msp.request(MSP_BOXIDS))                           # bit order of the MSP_STATUS flags
    try:
        while True:
            state = read_state(msp, box_ids)
            servos = struct.unpack("<8h", msp.request(MSP_SERVO)[:16])
            motors = struct.unpack("<8H", msp.request(MSP_MOTOR)[:16])
            servo_text = "  ".join("S%d(%s) %4d" % (index + 1, output, servos[index])
                                   for index, output in enumerate(SERVO_OUTPUTS))
            motor_text = "  ".join("M%d(%s) %4d" % (index + 1, output, motors[index])
                                   for index, output in enumerate(MOTOR_OUTPUTS))
            print("%-8s | servos %s | motors %s" % (state, servo_text, motor_text), flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
