#!/usr/bin/env python3
"""Servo driver on SITL: ADUM_GPS_PVT built from UBX NAV-PVT frames of a fake UBlox on the GPS UART.

  test/test_04_gps_pvt.py [--build] [--keep] [--verbose]

Scenarios:
 1. ADUM_GPS_PVT from UBX NAV-PVT: 3D fix, DGPS, RTK float, RTK fixed, 2D fix, no fix, unknown time/DOP

Exit code 0 when every check passes, 1 on a failed check, 2 when SITL cannot be started.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import harness as h  # noqa: E402
from common.harness import check, ubx_nav_pvt, expected_unix_usec  # noqa: E402

mav = h.mav
ADUM_TIME, ADUM_DIFF, ADUM_HEADING = (mav.ADUM_GPS_PVT_FLAGS_TIME_VALID, mav.ADUM_GPS_PVT_FLAGS_DIFF_CORRECTIONS,
                                    mav.ADUM_GPS_PVT_FLAGS_HEADING_VALID)


def s1_gps_pvt(fx):
    """ADUM_GPS_PVT from UBX NAV-PVT"""
    link, ublox = fx.link, fx.ublox
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


SCENARIOS = [s1_gps_pvt]

if __name__ == "__main__":
    sys.exit(h.run_scenarios(h.parse_args(__doc__), SCENARIOS, ublox=True))
