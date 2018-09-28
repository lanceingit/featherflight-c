#include "board.h"

#include "sensor.h"
#include "vector.h"

#define INS_MAX		3

static struct inertial_sensor_s* inertial_sensor[INS_MAX];
static uint8_t inertial_sensor_cnt=0;

static struct compass_s* compass[INS_MAX];
static uint8_t compass_cnt=0;

static struct baro_s* baro[INS_MAX];
static uint8_t baro_cnt=0;

void inertial_sensor_register(struct inertial_sensor_s* item)
{
	if(inertial_sensor_cnt >= INS_MAX) return;
    inertial_sensor[inertial_sensor_cnt++] = item;
}

void inertial_sensor_update(uint8_t ins)
{
	if(inertial_sensor_cnt > INS_MAX) return;
	inertial_sensor[ins]->update();
}

void inertial_sensor_get_acc(uint8_t ins, Vector* acc)
{
	if(inertial_sensor_cnt >= INS_MAX) return;
	*acc = inertial_sensor[ins]->acc;
}

void inertial_sensor_get_gyro(uint8_t ins, Vector* gyro)
{
	if(inertial_sensor_cnt >= INS_MAX) return;
	*gyro = inertial_sensor[ins]->gyro;
}

float inertial_sensor_get_acc_x(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return 0;
	return inertial_sensor[ins]->acc.x;
}

float inertial_sensor_get_acc_y(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return 0;
	return inertial_sensor[ins]->acc.y;
}

float inertial_sensor_get_acc_z(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return 0;
	return inertial_sensor[ins]->acc.z;
}

float inertial_sensor_get_gyro_x(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return 0;
	return inertial_sensor[ins]->gyro.x;
}

float inertial_sensor_get_gyro_y(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return 0;
	return inertial_sensor[ins]->gyro.y;
}

float inertial_sensor_get_gyro_z(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return 0;
	return inertial_sensor[ins]->gyro.z;
}

void inertial_sensor_set_gyro_offset_x(uint8_t ins, float f)
{
    inertial_sensor[ins]->gyro_offset.x = f;
}

void inertial_sensor_set_gyro_offset_y(uint8_t ins, float f)
{
    inertial_sensor[ins]->gyro_offset.y = f;
}

void inertial_sensor_set_gyro_offset_z(uint8_t ins, float f)
{
    inertial_sensor[ins]->gyro_offset.z = f;
}

bool inertial_sensor_ready(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return false;
	return inertial_sensor[ins]->ready;
}

void inertial_sensor_set_ready(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return;
	inertial_sensor[ins]->ready = true;
}

bool inertial_sensor_is_update(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return false;
	return inertial_sensor[ins]->is_update;
}

void inertial_sensor_clean_update(uint8_t ins)
{
	if(inertial_sensor_cnt >= INS_MAX) return;
	inertial_sensor[ins]->is_update = false;
}

void compass_register(struct compass_s* item)
{
	if(compass_cnt >= INS_MAX) return;
    compass[compass_cnt++] = item;	
}

void compass_update(uint8_t ins)
{
	if(compass_cnt >= INS_MAX) return;
	compass[ins]->update();
}

void compass_get_mag(uint8_t ins, Vector* mag)
{
	if(compass_cnt >= INS_MAX) return;
	*mag = compass[ins]->mag;
}

float compass_get_mag_x(uint8_t ins)
{
	if(compass_cnt >= INS_MAX) return 0;
	return compass[ins]->mag.x;
}

float compass_get_mag_y(uint8_t ins)
{
	if(compass_cnt >= INS_MAX) return 0;
	return compass[ins]->mag.y;
}

float compass_get_mag_z(uint8_t ins)
{
	if(compass_cnt >= INS_MAX) return 0;
	return compass[ins]->mag.z;
}

void baro_register(struct baro_s* item)
{
	if(baro_cnt >= INS_MAX) return;
    baro[baro_cnt++] = item;		
}

void baro_update(uint8_t ins)
{
	if(baro_cnt >= INS_MAX) return;
	baro[ins]->update();
}

float baro_get_press(uint8_t ins)
{
	if(baro_cnt >= INS_MAX) return 0;
	return baro[ins]->pressure;
}

float baro_get_altitude(uint8_t ins)
{
	if(baro_cnt >= INS_MAX) return 0;
	return baro[ins]->altitude;
}

float baro_get_temp(uint8_t ins)
{
	if(baro_cnt >= INS_MAX) return 0;
	return baro[ins]->temperature;
}

void sensor_init(void)
{
	uint8_t i;
	for(i=0; i<inertial_sensor_cnt; i++) {
		inertial_sensor[i]->init(ROTATION_ROLL_180_YAW_270);   //TODO:
	}
	for(i=0; i<compass_cnt; i++) {
		compass[i]->init();
	}
	for(i=0; i<baro_cnt; i++) {
		baro[i]->init();	
	}
}
