# MAVLink 2.0, ADUM dialect

Generated headers of the ADUM dialect (`adum.xml`, includes storm32, which includes ardupilotmega and common),
copied unchanged from the INAV fork, `lib/main/MAVLink` at commit `449206c32`. Do not edit:
change `adum.xml` in the INAV repository, regenerate there, and copy the result here.

Only the SATELLITEBOARD target uses it (its `target.mk` puts this directory first on the include path,
`#include "adum/mavlink.h"`). The MAVLink 1.0 headers in `lib/main/MAVLink` stay for the other targets.
`INAV_README.md` and `generate.sh` are the INAV notes on regenerating.
