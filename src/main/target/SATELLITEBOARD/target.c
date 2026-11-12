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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // Servos 1-5 (each timer has its own PWM rate, so servos and motors do not share one)
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_SERVO, 0),
    DEF_TIM(TIM2, CH2, PB3,  TIM_USE_SERVO, 0),
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_SERVO, 0),
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_SERVO, 0),
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_SERVO, 0),

    // Motors 1-3
    DEF_TIM(TIM3, CH1, PA6,  TIM_USE_MOTOR, 0),
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_MOTOR, 0),
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_MOTOR, 0),

    // PA7 is left unused: TIM3 CH2 would share the motor timer rate
    DEF_TIM(TIM3, CH2, PA7,  TIM_USE_NONE,  0),
};

// Pin remaps the PCB needs (the F103 default pins differ). Called from init() after systemInit(), before timers and UARTs.
void targetPreInit(void)
{
    // UART1 on PB6/PB7 instead of PA9/PA10
    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

    // TIM2 CH1/CH2 on PA15/PB3 instead of PA0/PA1 (PA2/PA3 stay UART2, CH3/CH4 are unused)
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

    // The remap calls above switch the debug port off: enable SWD again (JTAG stays off, PA15/PB3 stay usable)
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}
