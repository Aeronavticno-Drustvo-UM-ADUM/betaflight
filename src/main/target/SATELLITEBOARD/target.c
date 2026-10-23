#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_PWM,               TIMER_INPUT_ENABLED  ),
    DEF_TIM(TIM2, CH2, PB3,  TIM_USE_PWM,               TIMER_INPUT_ENABLED  ),
    DEF_TIM(TIM3, CH1, PA6,  TIM_USE_PWM,               TIMER_INPUT_ENABLED  ),
    DEF_TIM(TIM3, CH2, PA7,  TIM_USE_PWM,               TIMER_INPUT_ENABLED  ),
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_PWM,               TIMER_INPUT_ENABLED  ),
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_PWM,               TIMER_INPUT_ENABLED  ),
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED )
};