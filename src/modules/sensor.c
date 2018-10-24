#include "board.h"

#include "sensor.h"
#include "vector.h"

#define INS_MAX		3

static struct imu_s* imu[INS_MAX];
static uint8_t imu_cnt=0;

static struct compass_s* compass[INS_MAX];
static uint8_t compass_cnt=0;

static struct baro_s* baro[INS_MAX];
static uint8_t baro_cnt=0;

void imu_register(struct imu_s* item)
{
	if(imu_cnt >= INS_MAX) return;
    imu[imu_cnt++] = item;
}

void imu_update(uint8_t ins)
{
	if(ins > imu_cnt) return;
	imu[ins]->update();
}

void imu_get_acc(uint8_t ins, Vector* acc)
{
	if(ins > imu_cnt) return;
	*acc = imu[ins]->acc;
}

void imu_get_gyro(uint8_t ins, Vector* gyro)
{
	if(ins > imu_cnt) return;
	*gyro = imu[ins]->gyro;
}

float imu_get_acc_x(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->acc.x;
}

float imu_get_acc_y(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->acc.y;
}

float imu_get_acc_z(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->acc.z;
}

float imu_get_gyro_x(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->gyro.x;
}

float imu_get_gyro_y(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->gyro.y;
}

float imu_get_gyro_z(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->gyro.z;
}

float imu_get_temp(uint8_t ins)
{
	if(ins > imu_cnt) return 0;
	return imu[ins]->temp;
}

void imu_set_gyro_offset_x(uint8_t ins, float f)
{
	if(ins > imu_cnt) return;
    imu[ins]->gyro_offset.x = f;
}

void imu_set_gyro_offset_y(uint8_t ins, float f)
{
	if(ins > imu_cnt) return;
    imu[ins]->gyro_offset.y = f;
}

void imu_set_gyro_offset_z(uint8_t ins, float f)
{
	if(ins > imu_cnt) return;
    imu[ins]->gyro_offset.z = f;
}

bool imu_ready(uint8_t ins)
{
	if(ins > imu_cnt) return false;
	return imu[ins]->ready;
}

void imu_set_ready(uint8_t ins)
{
	if(ins > imu_cnt) return;
	imu[ins]->ready = true;
}

bool imu_is_update(uint8_t ins)
{
	if(ins > imu_cnt) return false;
	return imu[ins]->is_update;
}

void imu_clean_update(uint8_t ins)
{
	if(ins > imu_cnt) return;
	imu[ins]->is_update = false;
}

void compass_register(struct compass_s* item)
{
	if(compass_cnt >= INS_MAX) return;
    compass[compass_cnt++] = item;	
}

void compass_update(uint8_t ins)
{
	if(ins >= compass_cnt) return;
	compass[ins]->update();
}

void compass_get_mag(uint8_t ins, Vector* mag)
{
	if(ins >= compass_cnt) return;
	*mag = compass[ins]->mag;
}

float compass_get_mag_x(uint8_t ins)
{
	if(ins >= compass_cnt) return 0;
	return compass[ins]->mag.x;
}

float compass_get_mag_y(uint8_t ins)
{
	if(ins >= compass_cnt) return 0;
	return compass[ins]->mag.y;
}

float compass_get_mag_z(uint8_t ins)
{
	if(ins >= compass_cnt) return 0;
	return compass[ins]->mag.z;
}

void baro_register(struct baro_s* item)
{
	if(baro_cnt >= INS_MAX) return;
    baro[baro_cnt++] = item;		
}

void baro_update(uint8_t ins)
{
	if(ins >= baro_cnt) return;
	baro[ins]->update();
}

float baro_get_press(uint8_t ins)
{
	if(ins >= baro_cnt) return 0;
	return baro[ins]->pressure;
}

float baro_get_altitude(uint8_t ins)
{
	if(ins >= baro_cnt) return 0;
	return baro[ins]->altitude;
}

float baro_get_temp(uint8_t ins)
{
	if(ins >= baro_cnt) return 0;
	return baro[ins]->temperature;
}

void sensor_init(void)
{
	uint8_t i;
	for(i=0; i<imu_cnt; i++) {
		imu[i]->init(INERTIAL_SENSOR_ROTATION);   //TODO:
	}
	for(i=0; i<compass_cnt; i++) {
		compass[i]->init();
	}
	for(i=0; i<baro_cnt; i++) {
		baro[i]->init();	
	}
}
