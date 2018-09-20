#pragma once

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

	LowPassFilter2p	accel_filter_x;
	LowPassFilter2p	accel_filter_y;
	LowPassFilter2p	accel_filter_z;
	LowPassFilter2p	gyro_filter_x;
	LowPassFilter2p	gyro_filter_y;
	LowPassFilter2p	gyro_filter_z;

    //method
    init_func* init;
    read_func* read;

    read_status_func* ready;
    set_status_func* set_ready;

    get_float_func* get_acc_x;
    get_float_func* get_acc_y;
    get_float_func* get_acc_z;
    get_float_func* get_gyro_x;
    get_float_func* get_gyro_y;
    get_float_func* get_gyro_z;

    get_vector_3f_func* get_accel;
    get_vector_3f_func* get_gyro;
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

