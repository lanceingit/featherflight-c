#include "board.h"

#include "sensor.h"
#include "vector.h"
#include "timer.h"

#define INS_MAX		3

static struct imu_s* imu[INS_MAX];
static uint8_t imu_cnt=0;

static struct compass_s* compass[INS_MAX];
static uint8_t compass_cnt=0;

static struct baro_s* baro[INS_MAX];
static uint8_t baro_cnt=0;

void sensor_gyro_cal(void)         
{
#define GYRO_CAL_STATUS_COLLECT  	0
#define GYRO_CAL_STATUS_CHECK_MOVE  1
#define GYRO_CAL_STATUS_CAL  		2
#define GYRO_CAL_STATUS_IDEL  		3

#define COLLECT_MAX  50

	static uint8_t status = GYRO_CAL_STATUS_COLLECT;
	static uint8_t collect_cnt=0;

	Vector gyro;
	Vector gyro_sum;
	static Vector accel_start;
	Vector accel_end;
	Vector accel_diff;

	if(status == GYRO_CAL_STATUS_COLLECT) {
		// if(!armed) {
			// if(timer_check(cal_time, 50*1000)) {
				imu_get_acc(0, &accel_start);
				gyro_sum = vector_set(0,0,0);
				
				if(collect_cnt < COLLECT_MAX) {
					imu_get_gyro(0,&gyro);
					gyro_sum = vector_add(gyro_sum, gyro);
					collect_cnt++;
					//delay_ms(10);
				} else {
					status = GYRO_CAL_STATUS_CHECK_MOVE;
				}		
			// }
		// } else {
		// 	status = GYRO_CAL_STATUS_IDEL;
		// }
	} else if(status == GYRO_CAL_STATUS_CHECK_MOVE) {
		imu_get_acc(0,&accel_end);
		accel_diff = vector_sub(accel_start, accel_end);
		if(vector_length(accel_diff) >  0.2f) {
			status = GYRO_CAL_STATUS_COLLECT;
		} else {
			status = GYRO_CAL_STATUS_CAL;
		}	
	} else if(status == GYRO_CAL_STATUS_CAL) {
		imu_set_gyro_offset_x(0, gyro_sum.x/50);   
		imu_set_gyro_offset_y(0, gyro_sum.y/50);
		imu_set_gyro_offset_z(0, gyro_sum.z/50);
		status = GYRO_CAL_STATUS_IDEL;
	} else if(status == GYRO_CAL_STATUS_IDEL) {
		// if() {
		// 	status = GYRO_CAL_STATUS_COLLECT;
		// }
	}


	

}

void imu_register(struct imu_s* item)
{
	if(imu_cnt >= INS_MAX) return;
    imu[imu_cnt++] = item;
}

void imu_update(uint8_t ins)
{
	Vector acc; 
	Vector gyro;

	if(ins > imu_cnt) return;
	imu[ins]->update(&acc, &gyro);

    rotate_3f(imu[ins]->rotation, &acc.x, &acc.y, &acc.z);
    imu[ins]->acc.x = lpf_apply(&imu[ins]->acc_filter_x, acc.x);
    imu[ins]->acc.y = lpf_apply(&imu[ins]->acc_filter_y, acc.y);
    imu[ins]->acc.z = lpf_apply(&imu[ins]->acc_filter_z, acc.z);

    rotate_3f(imu[ins]->rotation, &gyro.x, &gyro.y, &gyro.z);
    gyro = vector_sub(gyro, imu[ins]->gyro_offset);

	imu[ins]->gyro.x = lpf_apply(&imu[ins]->gyro_filter_x, gyro.x);
	imu[ins]->gyro.y = lpf_apply(&imu[ins]->gyro_filter_y, gyro.y);
	imu[ins]->gyro.z = lpf_apply(&imu[ins]->gyro_filter_z, gyro.z);

	imu[ins]->is_update = true;
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
		lpf_init(&imu[i]->acc_filter_x, MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
		lpf_init(&imu[i]->acc_filter_y, MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
		lpf_init(&imu[i]->acc_filter_z, MPU6050_ACCEL_DEFAULT_RATE, MPU6050_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);
		lpf_init(&imu[i]->gyro_filter_x, MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
		lpf_init(&imu[i]->gyro_filter_y, MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ);
		lpf_init(&imu[i]->gyro_filter_z, MPU6050_GYRO_DEFAULT_RATE, MPU6050_GYRO_DEFAULT_DRIVER_FILTER_FREQ);

		imu[i]->rotation = INERTIAL_SENSOR_ROTATION;
		imu[i]->is_update = false;

		if(imu[i]->init()) {
			imu[i]->ready = true;   
		} 
	}
	for(i=0; i<compass_cnt; i++) {
		compass[i]->init();
	}
	for(i=0; i<baro_cnt; i++) {
		baro[i]->init();	
	}
}
