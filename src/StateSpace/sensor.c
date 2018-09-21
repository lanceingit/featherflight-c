#include "sensor.h"

static struct inertial_sensor_s* inertial_sensor;
static struct compass_s* compass;
static struct baro_s* baro;

void inertial_sensor_register(struct inertial_sensor_s* item)
{
    inertial_sensor = item;
}

void inertial_sensor_read(void)
{
	inertial_sensor->read();
}

void inertial_sensor_get_accel(float *acc)
{
	acc[0] = inertial_sensor->acc[0];
	acc[1] = inertial_sensor->acc[1];
	acc[2] = inertial_sensor->acc[2];
}

void inertial_sensor_get_gyro(float *gyro)
{
	gyro[0] = inertial_sensor->gyro[0];
	gyro[1] = inertial_sensor->gyro[1];
	gyro[2] = inertial_sensor->gyro[2];
}

float inertial_sensor_get_acc_x()
{
	return inertial_sensor->acc[0];
}

float inertial_sensor_get_acc_y()
{
	return inertial_sensor->acc[1];
}

float inertial_sensor_get_acc_z()
{
	return inertial_sensor->acc[2];
}

float inertial_sensor_get_gyro_x()
{
	return inertial_sensor->gyro[0];
}

float inertial_sensor_get_gyro_y()
{
	return inertial_sensor->gyro[1];
}

float inertial_sensor_get_gyro_z()
{
	return inertial_sensor->gyro[2];
}

bool inertial_sensor_ready(void)
{
	return inertial_sensor->ready;
}

void inertial_sensor_set_ready(void)
{
	inertial_sensor->ready = true;
}

void compass_register(struct compass_s* item)
{
    compass = item;
}

void compass_read(void)
{
	compass->read();
}

float compass_get_mag_x()
{
	return compass->mag[0];
}

float compass_get_mag_y()
{
	return compass->mag[1];
}

float compass_get_mag_z()
{
	return compass->mag[2];
}

void baro_register(struct baro_s* item)
{
    baro = item;
}

void baro_read(void)
{
	baro->read();
}

float baro_get_press()
{
	return baro->pressure;
}

float baro_get_altitude()
{
	return baro->altitude;
}

float baro_get_temp()
{
	return baro->temperature;
}

void sensor_init(void)
{
	inertial_sensor->init();
	compass->init();
	baro->init();	
}
