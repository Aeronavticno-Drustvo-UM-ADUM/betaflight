# ADUM MAVLink dialect

`adum.xml` is the ADUM private dialect. It includes `storm32.xml` (so it is a superset of
the dialect INAV used before) and is the primary dialect of this fork: `src/main/mavlink/mavlink_types.h`
and `src/main/rx/mavlink.h` include `adum/mavlink.h`.

Rules:
- Every message and enum is named `ADUM_*`.
- Message IDs **13000-13999** are reserved for this dialect (unused by every upstream v1.0 dialect).
  Take the next free ID in that block. Never reuse or renumber a released ID.
- The same `adum.xml` must be used by the GPS-RTK module firmware.

## Messages

| ID | Name | Direction | Purpose |
|---|---|---|---|
| 13000 | `ADUM_GPS_PVT` | GPS-RTK module -> FC | Position, velocity, time, accuracy, fix/RTK state |
| 13001 | `ADUM_MAGNETOMETER` | sensor -> FC | Magnetometer sample; same fields as MSP2_SENSOR_COMPASS (instance, time, X/Y/Z in mGauss: front, right, down) |
| 13002 | `ADUM_AUX_CHANNELS` | FC -> satellite board | `arming_status` (`ADUM_ARMING_STATUS`), `motor[3]` (motors 1-3), `servo[8]` (servos 1-8); values in us as INAV `motor[]`/`servo[]` |

`ADUM_GPS_PVT` units: `lat`/`lon` degE7, `alt_*` mm, `vel_*` mm/s, `heading_motion` degE5,
`*_acc` mm or mm/s, `pdop` x100. `fix_type` uses the standard `GPS_FIX_TYPE`
(0 NO_GPS, 1 NO_FIX, 2 2D_FIX, 3 3D_FIX, 4 DGPS, 5 RTK_FLOAT, 6 RTK_FIXED, 7 STATIC, 8 PPP). `flags` is `ADUM_GPS_PVT_FLAGS`.

## Regenerating

Edit `adum.xml`, then run `generate.sh` (see README.md). It copies `adum.xml` into the
mavlink checkout and generates all dialects. The committed `adum/` folder was generated
against mavlink commit `f9cb1f9e` (2026-07-06), whose message CRCs match the committed
`common/` and `storm32/` headers. If a later regeneration changes the other dialect
folders, review that diff separately.
