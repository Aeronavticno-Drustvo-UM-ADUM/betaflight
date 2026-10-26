#!/usr/bin/env python3
"""Minimal MAVLink v1 message generator (standard library only).

Builds the frames the SATELLITEBOARD servo driver understands: HEARTBEAT (arm flag),
COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM and RC_CHANNELS_OVERRIDE (channels 1-8).
The frame layout, field order and CRC extras match the headers in lib/main/MAVLink/common.
"""
import struct

STX_V1 = 0xFE

MSG_HEARTBEAT = 0
MSG_COMMAND_LONG = 76
MSG_RC_CHANNELS_OVERRIDE = 70

# msgid -> CRC_EXTRA (from lib/main/MAVLink/common/mavlink_msg_*.h)
CRC_EXTRA = {MSG_HEARTBEAT: 50, MSG_COMMAND_LONG: 152, MSG_RC_CHANNELS_OVERRIDE: 124}

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4

CHANNEL_IGNORE = 0xFFFF  # RC_CHANNELS_OVERRIDE: leave the channel unchanged


def _crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def mavlink_crc(data, extra):
    crc = 0xFFFF
    for byte in data:
        crc = _crc_accumulate(byte, crc)
    return _crc_accumulate(extra, crc)


class MavlinkGenerator:
    def __init__(self, sysid=255, compid=190):
        self.sysid = sysid
        self.compid = compid
        self.seq = 0

    def _frame(self, msgid, payload):
        header = struct.pack("<BBBBB", len(payload), self.seq, self.sysid, self.compid, msgid)
        self.seq = (self.seq + 1) & 0xFF
        crc = mavlink_crc(header + payload, CRC_EXTRA[msgid])
        return bytes([STX_V1]) + header + payload + struct.pack("<H", crc)

    def heartbeat(self, armed):
        base_mode = MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
        # custom_mode u32, type, autopilot, base_mode, system_status, mavlink_version
        payload = struct.pack("<IBBBBB", 0, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode, MAV_STATE_ACTIVE, 3)
        return self._frame(MSG_HEARTBEAT, payload)

    def arm_command(self, arm, target_system=1, target_component=0):
        # params 1-7 float, command u16, target_system, target_component, confirmation
        payload = struct.pack("<7fHBBB", 1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0,
                              MAV_CMD_COMPONENT_ARM_DISARM, target_system, target_component, 0)
        return self._frame(MSG_COMMAND_LONG, payload)

    def rc_override(self, channels, target_system=1, target_component=0):
        """channels: up to 8 pulse widths in us; None/0xFFFF = unchanged."""
        values = [CHANNEL_IGNORE if c is None else int(c) for c in channels]
        values += [CHANNEL_IGNORE] * (8 - len(values))
        payload = struct.pack("<8HBB", *values[:8], target_system, target_component)
        return self._frame(MSG_RC_CHANNELS_OVERRIDE, payload)


if __name__ == "__main__":
    gen = MavlinkGenerator()
    print("heartbeat (armed):", gen.heartbeat(True).hex())
    print("rc override      :", gen.rc_override([1200, 1500, 1800, 1500, 1500, 1100, 1100, 1100]).hex())
