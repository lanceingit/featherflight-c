#pragma once

#include "rotation.h"
#include "lpf.h"
#include "vector.h"
#include "func_type.h"


struct imu_s
{
    //member
    bool ready;    
    bool is_update;   

    Vector acc;
    Vector gyro;
    Vector gyro_offset;
    float temp;

	struct lpf_s	acc_filter_x;
	struct lpf_s	acc_filter_y;
	struct lpf_s	acc_filter_z;
	struct lpf_s	gyro_filter_x;
	struct lpf_s	gyro_filter_y;
	struct lpf_s	gyro_filter_z;

    enum Rotation rotation;

    //method
    init_func* init;
    imu_update_func* update;
};

struct compass_s
{
    //member
    Vector mag;
    
    //method
    init_func* init;
    update_func* update;
};

struct baro_s
{
    //member
    float temperature;
    float pressure;
    float altitude;

    //method
    init_func* init;
    update_func* update;     
};

void imu_register(struct imu_s* item);
void imu_update(uint8_t ins);
void imu_get_acc(uint8_t ins, Vector* acc);
void imu_get_gyro(uint8_t ins, Vector* gyro);
float imu_get_acc_x(uint8_t ins);
float imu_get_acc_y(uint8_t ins);
float imu_get_acc_z(uint8_t ins);
float imu_get_gyro_x(uint8_t ins);
float imu_get_gyro_y(uint8_t ins);
float imu_get_gyro_z(uint8_t ins);
float imu_get_temp(uint8_t ins);
void imu_set_gyro_offset_x(uint8_t ins, float f);
void imu_set_gyro_offset_y(uint8_t ins, float f);
void imu_set_gyro_offset_z(uint8_t ins, float f);
bool imu_ready(uint8_t ins);
void imu_set_ready(uint8_t ins);
bool imu_is_update(uint8_t ins);
void imu_clean_update(uint8_t ins);

void compass_register(struct compass_s* item);
void compass_update(uint8_t ins);
void compass_get_mag(uint8_t ins, Vector* mag);
float compass_get_mag_x(uint8_t ins);
float compass_get_mag_y(uint8_t ins);
float compass_get_mag_z(uint8_t ins);

void baro_register(struct baro_s* item);
void baro_update(uint8_t ins);
float baro_get_press(uint8_t ins);
float baro_get_altitude(uint8_t ins);
float baro_get_temp(uint8_t ins);

void sensor_init(void);

#ifdef F3_EVO
#include "mpu6050.h"
#include "hmc5883.h"
#include "ms5611.h"
#elif LINUX
#include "mpu6050_linux.h"
#include "spl06_linux.h"
#endif
