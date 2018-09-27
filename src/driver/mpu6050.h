#pragma once

#include "rotation.h"
#include "sensor.h"

struct mpu6050_s
{
	struct inertial_sensor_s heir;
	Vector gyro_raw;
    uint8_t buf[14];    
};

extern struct mpu6050_s mpu6050;

bool mpu6050_init(enum Rotation r);
void mpu6050_update(void);

bool mpu6050_is_update(void);
void mpu6050_clean_update(void);



