#pragma once

#define TARGET_BOARD_IDENTIFIER "SATBOARD"

#define LED0_PIN                PC15

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


#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C_DEVICE              (I2CDEV_1)
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9


#define DEFAULT_FEATURES        FEATURE_MOTOR_STOP

#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         ( BIT(13) | BIT(14) | BIT(15) )

#define USABLE_TIMER_CHANNEL_COUNT 9
#define USED_TIMERS             ( TIM_N(1) | TIM_N(2) | TIM_N(3) )