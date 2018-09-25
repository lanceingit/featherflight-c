#include <stdint.h>

#include "sensor.h"
#include "vector.h"

static struct inertial_sensor_s* inertial_sensor;
static struct compass_s* compass;
static struct baro_s* baro;

void inertial_sensor_register(struct inertial_sensor_s* item)
{
    inertial_sensor = item;
}

void inertial_sensor_update(void)
{
	inertial_sensor->update();
}

void inertial_sensor_get_acc(Vector* acc)
{
	*acc = vector_set(inertial_sensor->acc[0], inertial_sensor->acc[1], inertial_sensor->acc[2]);
}

void inertial_sensor_get_gyro(Vector* gyro)
{
	*gyro = vector_set(inertial_sensor->gyro[0], inertial_sensor->gyro[1], inertial_sensor->gyro[2]);
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

bool inertial_sensor_is_update(void)
{
	return inertial_sensor->is_update;
}

void inertial_sensor_clean_update(void)
{
	inertial_sensor->is_update = false;
}

void compass_register(struct compass_s* item)
{
    compass = item;
}

void compass_update(void)
{
	compass->update();
}

void compass_get_mag(Vector* mag)
{
	*mag = vector_set(compass->mag[0], compass->mag[1], compass->mag[2]);
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

void baro_update(void)
{
	baro->update();
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
	inertial_sensor->init(ROTATION_NONE);   //TODO:
	compass->init();
	baro->init();	
}
