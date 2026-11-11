#!/usr/bin/env bash
# Compare the frames of mavlink_gen.py byte for byte with the ones built by the firmware MAVLink headers.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

gcc -w -I "${ROOT}/lib/main/MAVLink_adum" -o "${TMP}/ref" "${HERE}/mavlink_ref.c" -lm
"${TMP}/ref" > "${TMP}/c.txt"
python3 -I - "${HERE}" > "${TMP}/py.txt" <<'PY'
import struct, sys
sys.path.insert(0, sys.argv[1])
from mavlink_gen import *
g = MavlinkGenerator()
print("hb:" + g.heartbeat(True).hex())
print("aux:" + g.aux_channels(True, [1500, 1600, 1700], [1100, 1200, 1300, 1400, 1500, None, None, None]).hex())
pvt = struct.pack("<Q8i3IHBBB", 1700000000123456, 473977000, 85432100, 430000, 477000, 1000, -2000, 300,
                  12345678, 2500, 3100, 450, 123, 6, 17, 7)
print("pvt:" + g._frame(MSG_ADUM_GPS_PVT, pvt).hex())
PY
if diff "${TMP}/c.txt" "${TMP}/py.txt"; then echo "mavlink_gen.py matches the firmware headers (3 frames)"; else echo "MISMATCH" >&2; exit 1; fi
