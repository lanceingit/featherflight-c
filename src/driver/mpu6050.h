#pragma once

#include "rotation.h"
#include "sensor.h"
#include "i2c.h"

struct mpu6050_s
{
	struct imu_s heir;
	Vector gyro_raw;
    uint8_t buf[14];    
	struct i2c_s* i2c;
};

extern struct mpu6050_s mpu6050;

bool mpu6050_init(enum Rotation r);
void mpu6050_update(void);

bool mpu6050_is_update(void);
void mpu6050_clean_update(void);



