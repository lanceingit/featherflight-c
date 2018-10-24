#include "board.h"
#include <fcntl.h>

#include "mpu6050_linux.h"
#include "lpf.h"
#include "sensor.h"

#define MPU6050_ACCEL_DEFAULT_RATE				1000
#define MPU6050_GYRO_DEFAULT_RATE				1000

#define MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30
#define MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define ACC_SCALE  0.002387768f 
#define GYRO_SCALE 0.001065185f 

struct mpu6050_linux_s mpu6050_linux = {
	.heir = {
		.init = &mpu6050_linux_init,
		.update = &mpu6050_linux_update,
	},
    .fd = -1,
};

static struct mpu6050_linux_s* this=&mpu6050_linux;


bool mpu6050_linux_init(enum Rotation r)
{
	lpf_init(&this->heir.acc_filter_x, MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	lpf_init(&this->heir.acc_filter_y, MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	lpf_init(&this->heir.acc_filter_z, MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
	lpf_init(&this->heir.gyro_filter_x, MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
	lpf_init(&this->heir.gyro_filter_y, MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
	lpf_init(&this->heir.gyro_filter_z, MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
	this->heir.ready = false;
	this->heir.rotation = r;
	this->heir.is_update = false;
	
	this->fd = open(MPU6050_PATH, O_RDONLY);
	if(this->fd < 0){
		return false;
	}

    this->heir.ready = true;
    
	return true;
}



void mpu6050_linux_update(void)
{
	if(read(this->fd, &this->report, sizeof(this->report)) <= 0) return;	

	this->heir.temp = 36.53f + (float)(this->report.temp) / 340.0f;

    Vector acc_new = {
        (float)(this->report.acc[0]* ACC_SCALE),
        (float)(this->report.acc[1]* ACC_SCALE),
        (float)(this->report.acc[2]* ACC_SCALE),
    };
    rotate_3f(this->heir.rotation, &acc_new.x, &acc_new.y, &acc_new.z);

    this->heir.acc.x = lpf_apply(&this->heir.acc_filter_x, acc_new.x);
    this->heir.acc.y = lpf_apply(&this->heir.acc_filter_y, acc_new.y);
    this->heir.acc.z = lpf_apply(&this->heir.acc_filter_z, acc_new.z);
    
    Vector gyro_new = {
        (float)(this->report.gyro[0]* GYRO_SCALE),
        (float)(this->report.gyro[1]* GYRO_SCALE),
        (float)(this->report.gyro[2]* GYRO_SCALE),
    };
    rotate_3f(this->heir.rotation, &gyro_new.x, &gyro_new.y, &gyro_new.z);

    this->gyro_raw = gyro_new;
    
    gyro_new = vector_sub(gyro_new, this->heir.gyro_offset);
    
	this->heir.gyro.x = lpf_apply(&this->heir.gyro_filter_x, gyro_new.x);
	this->heir.gyro.y = lpf_apply(&this->heir.gyro_filter_y, gyro_new.y);
	this->heir.gyro.z = lpf_apply(&this->heir.gyro_filter_z, gyro_new.z);

	this->heir.is_update = true;
}



