/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SATB"

// Servo driver only: no real sensors, config over MSP, MAVLink telemetry
#define LED0_PIN                PC15

// Servo driver mode: no gyro/PID/RX tasks, servos are driven from MAVLink (see io/servo_driver.c)
#define USE_SERVO_DRIVER
#define USE_FAKE_GYRO
#define USE_FAKE_ACC
#define USE_VCP
#define USE_UART1
#define USE_UART2
#define USE_UART3

#define SERIAL_PORT_COUNT       4

#define UART1_RX_PIN            PB7
#define UART1_TX_PIN            PB6

#define UART2_RX_PIN            PA3
#define UART2_TX_PIN            PA2

#define UART3_RX_PIN            PB11
#define UART3_TX_PIN            PB10

// Features dropped to fit 128 KB flash / 20 KB RAM
#undef USE_GYRO_REGISTER_DUMP
#undef USE_IMU_CALC
#undef USE_PPM
#undef USE_SERIALRX_CRSF
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_BRUSHED_ESC_AUTODETECT
#undef USE_ACRO_TRAINER
#undef USE_BLACKBOX
#undef USE_CLI_BATCH
#undef USE_RUNAWAY_TAKEOFF
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_SMARTPORT
#undef USE_RESOURCE_MGMT

// GPS: only enabled by common_pre.h for >256 KB parts
#define USE_GPS
#define USE_GPS_UBLOX

// Only enabled by common_pre.h for >256 KB parts
//#define USE_TELEMETRY_MAVLINK

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) )
