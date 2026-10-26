SITL_TARGETS += $(TARGET)
FEATURES       += #SDCARD_SPI VCP

# MAVLink 2.0 ADUM dialect (used by io/servo_driver.c when USE_SERVO_DRIVER is defined)
INCLUDE_DIRS := $(ROOT)/lib/main/MAVLink_adum $(INCLUDE_DIRS)

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c \
            io/servo_driver.c
