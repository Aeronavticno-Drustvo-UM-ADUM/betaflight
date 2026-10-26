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

// MAVLink 2.0 with the ADUM dialect (lib/main/MAVLink_adum), sized for the F103:
// - one channel buffer instead of the default four (about 800 bytes of RAM)
// - the message table only holds what the board receives; the default one lists about 330 messages (about 4 KB of flash)
//   Frames of any other message fail the table lookup and are dropped.

#include "common/maths.h"       // cos_approx / sin_approx used by mavlink_conversions.h

#define MAVLINK_COMM_NUM_BUFFERS 1

// {msgid, crc_extra, min_len, max_len, flags, target_system_ofs, target_component_ofs}
// The values are checked against the generated headers below.
#define MAVLINK_MESSAGE_CRCS { \
    {0, 50, 9, 9, 0, 0, 0},         /* HEARTBEAT */ \
    {13002, 190, 23, 23, 0, 0, 0},  /* ADUM_AUX_CHANNELS */ \
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "adum/mavlink.h"
#pragma GCC diagnostic pop

_Static_assert(MAVLINK_MSG_ID_HEARTBEAT == 0 && MAVLINK_MSG_ID_HEARTBEAT_CRC == 50
    && MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN == 9 && MAVLINK_MSG_ID_HEARTBEAT_LEN == 9,
    "MAVLINK_MESSAGE_CRCS: HEARTBEAT entry does not match the dialect");
_Static_assert(MAVLINK_MSG_ID_ADUM_AUX_CHANNELS == 13002 && MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_CRC == 190
    && MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_MIN_LEN == 23 && MAVLINK_MSG_ID_ADUM_AUX_CHANNELS_LEN == 23,
    "MAVLINK_MESSAGE_CRCS: ADUM_AUX_CHANNELS entry does not match the dialect");
