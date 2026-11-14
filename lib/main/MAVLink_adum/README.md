# MAVLink 2.0, ADUM dialect

Generated headers of the ADUM dialect (`adum.xml`, includes storm32, which includes ardupilotmega and common),
copied unchanged from the INAV fork, `lib/main/MAVLink` (last sync 2026-11-14: arming_status MOTOR_TEST). Do not edit:
change `adum.xml` in the INAV repository, regenerate there, and copy the result here (`adum.xml`, `adum/`,
`ADUM_DIALECT.md`). Check: `diff -rq ../inav/lib/main/MAVLink lib/main/MAVLink_adum` lists only README.md and the
INAV-only generator files.

Only the SATELLITEBOARD target uses it (its `target.mk` puts this directory first on the include path,
`#include "adum/mavlink.h"`). The MAVLink 1.0 headers in `lib/main/MAVLink` stay for the other targets.
Regeneration notes: INAV `lib/main/MAVLink/README.md` and `generate.sh`.
