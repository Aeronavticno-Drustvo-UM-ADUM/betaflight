/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"

#include "pg/pg.h"

// Servo driver mode: the flight controller core (gyro, PID, RX, mixer) is not run.
// The master flight controller sends MAVLink 2.0 ADUM_AUX_CHANNELS (arming status, servo[8], motor[3] in us).
// Arming and safety rules: docs/arming_safety.md (states DISARMED / ARMED / FAILSAFE, link sessions, arm-edge rule).
// - servos 1-5 always follow valid frames in every state (500-2500 us clamp), they hold the last value in failsafe
// - motors 1-3 follow armed frames only, minimum command otherwise; written every task cycle
// - no valid ADUM_AUX_CHANNELS for sd_link_timeout_ms while armed: FAILSAFE (never times out)
// The board answers with HEARTBEAT, SYS_STATUS, STATUSTEXT and ADUM_GPS_PVT.

typedef struct servoDriverConfig_s {
    uint16_t linkTimeoutMs;     // sd_link_timeout_ms: no valid ADUM_AUX_CHANNELS for this long while armed -> FAILSAFE
    uint16_t bootGraceMs;       // sd_boot_grace_ms: no arm edge accepted before this time since boot (ESC initialisation)
} servoDriverConfig_t;

PG_DECLARE(servoDriverConfig_t, servoDriverConfig);

#define SERVO_DRIVER_SERVO_COUNT   5
#define SERVO_DRIVER_MOTOR_COUNT   3

void servoDriverInit(void);
void servoDriverUpdate(timeUs_t currentTimeUs);
bool servoDriverIsArmed(void);

// True while ARMED or FAILSAFE: config actions (CLI entry, save, reboot, defaults, motor test, MSP set motor) are refused
bool servoDriverConfigLocked(void);
// Call on every MSP write command and motor test: arming checks refuse an arm edge for 2 s afterwards
void servoDriverNoteConfigActivity(void);
// Call around the EEPROM save: arming checks refuse an arm edge while it runs
void servoDriverSetSaveBusy(bool busy);
// Feeds the independent watchdog (F103 only, no-op elsewhere): call inside long flash operations
void servoDriverWatchdogFeed(void);
