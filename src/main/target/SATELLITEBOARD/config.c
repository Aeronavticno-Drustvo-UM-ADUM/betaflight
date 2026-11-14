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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "config/feature.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "io/gps.h"
#include "io/serial.h"
#include "pg/motor.h"

// Sets one UART's function and baud rates in the default serial config
static void setPortDefaults(serialPortIdentifier_e identifier, serialPortFunction_e function, baudRate_e baudRate)
{
    const int portIndex = findSerialPortIndexByIdentifier(identifier);
    if (portIndex < 0) {
        return;
    }
    serialPortConfig_t *port = &serialConfigMutable()->portConfigs[portIndex];
    port->functionMask = function;
    port->telemetry_baudrateIndex = baudRate;
    port->gps_baudrateIndex = baudRate;
}

// Board defaults (owner, 2026-11-14): the satellite works without any saved config, so a missing or broken
// config still gives the MAVLink link to the master. Same content as test/servo_config_board.txt, motors on PWM.
void targetConfiguration(void)
{
    setPortDefaults(SERIAL_PORT_USART1, FUNCTION_TELEMETRY_MAVLINK, BAUD_115200);   // link to the master
    setPortDefaults(SERIAL_PORT_USART2, FUNCTION_GPS, BAUD_115200);                 // GPS connector

    featureConfigSet(FEATURE_GPS);
    gpsConfigMutable()->provider = GPS_UBLOX;
    gpsConfigMutable()->autoConfig = GPS_AUTOCONFIG_ON;

    // Standard PWM for now (OneShot125 / DShot later); Betaflight's default is "disabled"
    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_STANDARD;
    motorConfigMutable()->dev.motorPwmRate = 50;                                    // standard PWM, 20 ms period
    motorConfigMutable()->mincommand = 1000;
    motorConfigMutable()->maxthrottle = 2000;
}

#endif // USE_TARGET_CONFIG
