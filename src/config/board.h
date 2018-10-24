#pragma once

#include "chip.h"
#include "version.h"

#ifdef F3_EVO

#define MPU6050_I2C     I2C1
#define HMC5883_I2C     I2C1
#define MS5611_I2C      I2C1

#define SPI_FLAHS_SPI   SPI2

#define INERTIAL_SENSOR_ROTATION    ROTATION_ROLL_180_YAW_270


#elif LINUX

#define MPU6050_PATH "/dev/mpu6050"
#define INERTIAL_SENSOR_ROTATION    ROTATION_ROLL_180_YAW_90

#define SPL06_PATH "/dev/spl06"

#endif



