F427_TARGETS    += $(TARGET)
FEATURES        += SDCARD VCP

HSE_VALUE = 24000000

TARGET_SRC = \
            drivers/accgyro_spi_mpu9250.c \
            drivers/compass_ak8963.c
