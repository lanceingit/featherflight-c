#pragma once

#include "rotation.h"
#include "LowPassFilter2p.h"


typedef float (get_float_func)(void);
typedef void (get_vector_3f_func)(float v[3]);
typedef bool (init_func)(void);
typedef void (read_func)(void);
typedef bool (read_status_func)(void);
typedef void (set_status_func)(void);


struct inertial_sensor_s
{
    //member
    bool ready;    

    float acc[3];
    float gyro[3];

	struct LowPassFilter2p	accel_filter_x;
	struct LowPassFilter2p	accel_filter_y;
	struct LowPassFilter2p	accel_filter_z;
	struct LowPassFilter2p	gyro_filter_x;
	struct LowPassFilter2p	gyro_filter_y;
	struct LowPassFilter2p	gyro_filter_z;

    enum Rotation rotation;

    //method
    init_func* init;
    read_func* read;
};

struct compass_s
{
    //member
    float mag[3];
    
    //method
    init_func* init;
    read_func* read;

    get_float_func* get_mag_x;
    get_float_func* get_mag_y;
    get_float_func* get_mag_z;

    get_vector_3f_func* get_mag;
};

struct baro_s
{
    //member
    float temperature;
    float pressure;
    float altitude;

    //method
    init_func* init;
    read_func* read;

    get_float_func* get_press;
    get_float_func* get_altitude;
    get_float_func* get_temp;       
};

void inertial_sensor_register(struct inertial_sensor_s* item);
void inertial_sensor_get_acc(float *acc);
void inertial_sensor_get_gyro(float *gyro);
float inertial_sensor_get_acc_x(void);
float inertial_sensor_get_acc_y(void);
float inertial_sensor_get_acc_z(void);
float inertial_sensor_get_gyro_x(void);
float inertial_sensor_get_gyro_y(void);
float inertial_sensor_get_gyro_z(void);
bool inertial_sensor_ready(void);
void inertial_sensor_set_ready(void);



void compass_register(struct compass_s* item);
void baro_register(struct baro_s* item);

