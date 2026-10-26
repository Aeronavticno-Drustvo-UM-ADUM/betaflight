F1_TARGETS  += $(TARGET)
FEATURES    = VCP

# MAVLink 2.0 ADUM dialect headers come first so that "adum/mavlink.h" is found (the v1 headers stay for other targets)
INCLUDE_DIRS := $(ROOT)/lib/main/MAVLink_adum $(INCLUDE_DIRS)

TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            io/servo_driver.c
