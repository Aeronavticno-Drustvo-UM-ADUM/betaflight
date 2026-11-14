#!/usr/bin/env python3
"""Servo driver on SITL: damaged, unusual and foreign MAVLink frames.

  test/test_03_frame_integrity.py [--build] [--keep] [--verbose]

Scenarios:
 1. Bad CRC: ignored, does not refresh the failsafe timeout
 2. Random garbage between valid frames: next valid frames applied, no crash
 3. Frame delivered byte by byte (1 ms gaps)
 4. Truncated frame followed by valid frames: parser recovery, N frames until the first one is applied
 5. Burst of 20 frames in one TCP write: last value wins
 6. Unknown message id, MAVLink 1 frame, wrong CRC_EXTRA: ignored, no effect on outputs or timeout
 7. Trailing zero bytes of the payload trimmed (MAVLink 2): decoded correctly
 8. Frame from another sysid/compid: accepted (no other nodes on the bus)
 9. Signed frame (incompat flag 0x01, 13 byte signature): observed behaviour is reported

Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common.harness import RUN, POSE, IDLE, check, known_issue  # noqa: E402

mav = h.mav
POSE_B = [1100, 1900, 1500, 1400, 1300, 1650]      # differs from POSE in every servo
RUN_B = [1250, 1350, 1450]
recovery_frames = []                          # N of every truncated-frame experiment


def motors_seen(fx, since, until=float("inf")):
    return set().union(*(fx.sitl.seen("motor", i, since, until) for i in range(h.MOTORS)))


def aux_frame(fx, armed, motors, servos, arming_raw=None):
    """Frame bytes without sending; the sequence number advances."""
    return fx.link.gen.aux_channels(armed, motors, servos, arming_raw)


def s1_bad_crc(fx):
    """Bad CRC frames are ignored and do not refresh the timeout"""
    fx.settle(True)
    t0 = time.monotonic()
    for _ in range(10):
        fx.link.send(h.corrupt_crc(aux_frame(fx, True, RUN_B, POSE_B)))
        time.sleep(0.02)
    time.sleep(0.1)
    check("bad CRC frames: outputs unchanged", fx.servos() == POSE and fx.motors() == RUN, "%s %s" % (fx.servos(), fx.motors()))
    check("bad CRC frames: no output event", not any(fx.sitl.changes("servo", i, t0) for i in range(h.SERVOS))
          and not any(fx.sitl.changes("motor", i, t0) for i in range(h.MOTORS)))
    fx.settle(True)
    t_last = fx.link.last_aux_t
    end = t_last + 1.2
    while time.monotonic() < end:
        fx.link.send(h.corrupt_crc(aux_frame(fx, True, RUN, POSE)))
        time.sleep(0.02)
    times = fx.disarm_times(t_last)
    ok = all(t is not None and 0.495 <= t - t_last <= 0.6 for t in times)
    check("only bad CRC frames: disarm 495-600 ms after the last good frame", ok,
          "delays ms %s" % [None if t is None else round((t - t_last) * 1e3) for t in times])


def garbage_round(fx, garbage, frames_after):
    """Sends the garbage, then frames_after valid frames at 50 Hz; returns how many of them were lost."""
    fx.settle(True)
    fx.link.send(bytes(garbage))
    t0 = time.monotonic()
    values = [1100 + 40 * k for k in range(frames_after)]            # 1100 ... no 0xFD byte
    for value in values:
        fx.link.send_aux(True, RUN, [value] + POSE[1:])
        time.sleep(0.02)
    time.sleep(0.1)
    applied = [v for v in fx.sitl.changes("servo", 0, t0) if v in values]
    return len(values) - len(applied), fx.servos()[0] == values[-1]


def s2_garbage(fx):
    """Random garbage between valid frames"""
    random = h.rng(20261026)
    # A) garbage whose last 270 bytes have no start byte (0xFD, 0xFE): every frame the garbage may have started is
    #    finished before the valid frames follow, so at most the first valid frame may be lost.
    for rnd in range(3):
        garbage = bytearray(random.randrange(256) for _ in range(1000))
        for pos in random.sample(range(700), 20):                    # plenty of 0xFD start bytes
            garbage[pos] = mav.STX_V2
        for pos in range(730, 1000):
            if garbage[pos] in (0xFD, 0xFE):
                garbage[pos] = 0x55
        lost, last_ok = garbage_round(fx, garbage, 5)
        check("clean tail, round %d: valid frames applied, at most 1 lost" % (rnd + 1), last_ok and lost <= 1,
              "lost %d of 5" % lost)
    # B) plain random garbage with 0xFD in it: a start byte near its end may swallow the next valid bytes (up to one
    #    maximum length frame, 267 bytes = 8 frames of 35 bytes). Recovery must happen within 9 frames.
    for rnd in range(5):
        garbage = bytearray(random.randrange(256) for _ in range(1000))
        for pos in random.sample(range(len(garbage) - 40), 20):
            garbage[pos] = mav.STX_V2
        lost, last_ok = garbage_round(fx, garbage, 14)
        check("random tail, round %d: recovers within 9 frames" % (rnd + 1), last_ok and lost <= 9, "lost %d of 14" % lost)
    check("SITL alive after the garbage", fx.sitl.alive())
    fx.link.frames(0.3, True, RUN, POSE)
    check("board still works after the garbage", fx.servos() == POSE and fx.motors() == RUN)


def s3_byte_by_byte(fx):
    """Frame delivered byte by byte with 1 ms gaps"""
    fx.settle(True)
    for k, pose in enumerate((POSE_B, [1111, 1222, 1333, 1444, 1555, 1666], POSE)):
        frame = aux_frame(fx, True, RUN_B, pose)
        t0 = time.monotonic()
        for byte in frame:
            fx.link.sock.sendall(bytes([byte]))
            time.sleep(0.001)
        fx.link.last_aux_t = time.monotonic()
        wait = fx.wait_outputs(servos=pose, motors=RUN_B)
        check("frame %d (%d bytes in %.0f ms) applied" % (k + 1, len(frame), (time.monotonic() - t0) * 1e3),
              wait is not None, "servos %s motors %s" % (fx.servos(), fx.motors()))


def s4_truncated(fx):
    """Truncated frame followed by valid frames"""
    for cut in (1, 10, 25):
        fx.settle(True)
        fx.link.send(aux_frame(fx, True, RUN, POSE)[:-cut])
        t0 = time.monotonic()
        values = [1100 + 40 * k for k in range(6)]
        for value in values:
            fx.link.send_aux(True, RUN, [value] + POSE[1:])
            time.sleep(0.02)
        time.sleep(0.1)
        applied = fx.sitl.changes("servo", 0, t0)
        first = next((k + 1 for k, v in enumerate(values) if v in applied), None)
        if first is not None:
            recovery_frames.append(first)
        check("truncated by %d bytes: recovers within 2 frames" % cut, first is not None and first <= 2,
              "first applied frame N = %s" % first)
        check("truncated by %d bytes: later frames all applied" % cut, fx.servos()[0] == values[-1], str(fx.servos()[0]))


def s5_burst(fx):
    """20 frames in one TCP write, last value wins"""
    fx.settle(True)
    values = [1100 + 20 * k for k in range(20)]
    burst = b"".join(aux_frame(fx, True, RUN, [v] + POSE[1:]) for v in values)
    t0 = time.monotonic()
    fx.link.send_aux_frame(burst)
    time.sleep(0.5)
    applied = [v for v in fx.sitl.changes("servo", 0, t0) if v in values]
    check("last value of the burst wins", fx.servos()[0] == values[-1], "servo 1 = %s, %d of 20 values passed" % (fx.servos()[0], len(applied)))
    check("motors unchanged by the burst", motors_seen(fx, t0) == set(RUN))


def s6_foreign_frames(fx):
    """Unknown id, MAVLink 1 and wrong CRC_EXTRA frames are ignored"""
    unknown = h.raw_frame_v2(13050, bytes(range(1, 20)), 77)
    mav1_hb = h.raw_frame_v1(0, b"\x00" * 4 + bytes([0, 0, 0x80, 4, 3]), 50)
    mav1_aux_like = h.raw_frame_v1(200, h.aux_payload(True, RUN_B, POSE_B), 190)
    wrong_extra = h.raw_frame_v2(mav.MSG_ADUM_AUX_CHANNELS, h.aux_payload(True, RUN_B, POSE_B), 191)
    for name, junk in (("unknown message id", unknown), ("MAVLink 1 heartbeat", mav1_hb),
                       ("MAVLink 1 frame", mav1_aux_like), ("wrong CRC_EXTRA", wrong_extra)):
        fx.settle(True)
        t0 = time.monotonic()
        for _ in range(10):
            fx.link.send(junk)
            fx.link.send_aux(True, RUN, POSE)
            time.sleep(0.02)
        check("%s between valid frames: no effect on the outputs" % name,
              not any(fx.sitl.changes("servo", i, t0) for i in range(h.SERVOS)) and motors_seen(fx, t0) == set(RUN))
        fx.settle(True)
        t_last = fx.link.last_aux_t
        while time.monotonic() < t_last + 1.2:
            fx.link.send(junk)
            time.sleep(0.02)
        times = fx.disarm_times(t_last)
        ok = all(t is not None and 0.495 <= t - t_last <= 0.6 for t in times)
        check("%s only: does not refresh the timeout (disarm 495-600 ms)" % name, ok,
              "delays ms %s" % [None if t is None else round((t - t_last) * 1e3) for t in times])


def s7_trimmed_payload(fx):
    """MAVLink 2 trailing zero bytes of the payload trimmed"""
    # wire order is motor[3], servo[8], arming_status: an armed frame always ends with the non-zero arming byte,
    # so only disarmed frames can be trimmed (arming_status 0 and the trailing zero servo values drop out).
    fx.settle(True)
    frame = aux_frame(fx, False, [0, 0, 0], [1100, 1200, 1300, 1400, 1500, 1600, 0, 0])
    check("frame is really trimmed (payload < 23 bytes)", frame[1] < 23, "payload %d bytes" % frame[1])
    fx.link.send_aux_frame(frame)
    wait = fx.wait_outputs(servos=[1100, 1200, 1300, 1400, 1500, 1600], motors=IDLE)
    check("trimmed disarmed frame: servos applied, missing arming byte = disarmed", wait is not None,
          "servos %s motors %s" % (fx.servos(), fx.motors()))
    fx.settle(True)
    frame = aux_frame(fx, False, [0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0])
    check("all-zero frame trimmed to %d byte(s)" % frame[1], frame[1] < 23)
    fx.link.send_aux_frame(frame)
    wait = fx.wait_outputs(motors=IDLE)
    check("all-zero trimmed frame: disarmed, servos keep their position", wait is not None and fx.servos() == POSE,
          "servos %s motors %s" % (fx.servos(), fx.motors()))
    fx.settle(True)
    frame = aux_frame(fx, False, RUN, [1111, 1222, 1333, None, None, None, None, None])
    fx.link.send_aux_frame(frame)
    wait = fx.wait_outputs(servos=[1111, 1222, 1333, POSE[3], POSE[4], POSE[5]], motors=IDLE)
    check("full-length disarmed frame still works", wait is not None and fx.servos()[:3] == [1111, 1222, 1333],
          str(fx.servos()))
    # armed frame whose payload is cut right after the servos: arming_status missing = 0 = disarmed
    fx.settle(True)
    payload = h.aux_payload(True, RUN_B, POSE_B)[:-1]
    fx.link.send_aux_frame(h.raw_frame_v2(mav.MSG_ADUM_AUX_CHANNELS, payload, 190))
    wait = fx.wait_outputs(motors=IDLE, servos=POSE_B)
    check("payload without the arming_status byte: disarmed, servos applied", wait is not None,
          "servos %s motors %s" % (fx.servos(), fx.motors()))


def s8_other_ids(fx):
    """Frames from another sysid/compid are accepted"""
    fx.settle(False)
    for sysid, compid in ((42, 200), (0, 0), (255, 190)):
        gen = mav.MavlinkGenerator(sysid, compid)
        pose = [1100 + sysid, 1200 + sysid, 1300, 1400, 1500, 1600]
        fx.link.send_aux_frame(gen.aux_channels(True, RUN_B, pose))
        wait = fx.wait_outputs(servos=pose, motors=RUN_B)
        check("sysid %d compid %d accepted" % (sysid, compid), wait is not None, "servos %s motors %s" % (fx.servos(), fx.motors()))
    fx.settle(False)


def s9_signed(fx):
    """Signed frame (incompat flag 0x01 + 13 byte signature)"""
    fx.settle(True)
    signed = h.raw_frame_v2(mav.MSG_ADUM_AUX_CHANNELS, h.aux_payload(True, RUN_B, POSE_B), 190, incompat=0x01,
                            signature=bytes(range(1, 14)))
    t0 = time.monotonic()
    fx.link.send(signed)
    wait = fx.wait_outputs(servos=POSE_B, motors=RUN_B, timeout=0.3)
    if wait is not None:
        known_issue("signed frames are accepted without checking the signature", "outputs followed after %d ms" % round(wait * 1e3))
    else:
        check("signed frame ignored", True, "servos %s motors %s" % (fx.servos(), fx.motors()))
    time.sleep(0.2)
    fx.link.frames(0.3, True, RUN, POSE)
    check("unsigned frames after the signed one are applied (signature bytes consumed)",
          fx.servos() == POSE and fx.motors() == RUN, "servos %s motors %s" % (fx.servos(), fx.motors()))
    del t0


SCENARIOS = [s1_bad_crc, s2_garbage, s3_byte_by_byte, s4_truncated, s5_burst, s6_foreign_frames, s7_trimmed_payload,
             s8_other_ids, s9_signed]

if __name__ == "__main__":
    code = h.run_scenarios(h.parse_args(__doc__), SCENARIOS)
    if recovery_frames:
        print("parser recovery after a truncated frame: N = %s" % recovery_frames)
    sys.exit(code)
