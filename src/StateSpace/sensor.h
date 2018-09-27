#pragma once

#include <stdbool.h>
#include "rotation.h"
#include "LowPassFilter2p.h"
#include "vector.h"
#include "func_type.h"


struct inertial_sensor_s
{
    //member
    bool ready;    
    bool is_update;   

    Vector acc;
    Vector gyro;

	struct LowPassFilter2p	acc_filter_x;
	struct LowPassFilter2p	acc_filter_y;
	struct LowPassFilter2p	acc_filter_z;
	struct LowPassFilter2p	gyro_filter_x;
	struct LowPassFilter2p	gyro_filter_y;
	struct LowPassFilter2p	gyro_filter_z;

    enum Rotation rotation;

    //method
    init_func_rocation* init;
    update_func* update;
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

void inertial_sensor_register(struct inertial_sensor_s* item);
void inertial_sensor_update(uint8_t ins);
void inertial_sensor_get_acc(uint8_t ins, Vector* acc);
void inertial_sensor_get_gyro(uint8_t ins, Vector* gyro);
float inertial_sensor_get_acc_x(uint8_t ins);
float inertial_sensor_get_acc_y(uint8_t ins);
float inertial_sensor_get_acc_z(uint8_t ins);
float inertial_sensor_get_gyro_x(uint8_t ins);
float inertial_sensor_get_gyro_y(uint8_t ins);
float inertial_sensor_get_gyro_z(uint8_t ins);
bool inertial_sensor_ready(uint8_t ins);
void inertial_sensor_set_ready(uint8_t ins);
bool inertial_sensor_is_update(uint8_t ins);
void inertial_sensor_clean_update(uint8_t ins);

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


#include "mpu6050.h"
#include "hmc5883.h"
#include "ms5611.h"
