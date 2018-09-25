#pragma once

#include <stdbool.h>
#include "rotation.h"
#include "LowPassFilter2p.h"
#include "vector.h"


typedef float (get_float_func)(void);
typedef void (get_vector_3f_func)(float v[3]);
typedef bool (init_func)(void);
typedef bool (init_func_rocation)(enum Rotation r);
typedef void (update_func)(void);
typedef bool (read_status_func)(void);
typedef void (set_status_func)(void);


struct inertial_sensor_s
{
    //member
    bool ready;    
    bool is_update;   

    float acc[3];
    float gyro[3];

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
    float mag[3];
    
    //method
    init_func* init;
    update_func* update;

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
    update_func* update;

    get_float_func* get_press;
    get_float_func* get_altitude;
    get_float_func* get_temp;       
};

void inertial_sensor_register(struct inertial_sensor_s* item);
void inertial_sensor_update(void);
void inertial_sensor_get_acc(Vector* acc);
void inertial_sensor_get_gyro(Vector* gyro);
float inertial_sensor_get_acc_x(void);
float inertial_sensor_get_acc_y(void);
float inertial_sensor_get_acc_z(void);
float inertial_sensor_get_gyro_x(void);
float inertial_sensor_get_gyro_y(void);
float inertial_sensor_get_gyro_z(void);
bool inertial_sensor_ready(void);
void inertial_sensor_set_ready(void);
bool inertial_sensor_is_update(void);
void inertial_sensor_clean_update(void);

void compass_register(struct compass_s* item);
void compass_update(void);
void compass_get_mag(Vector* mag);
float compass_get_mag_x(void);
float compass_get_mag_y(void);
float compass_get_mag_z(void);

void baro_register(struct baro_s* item);
void baro_update(void);
float baro_get_press(void);
float baro_get_altitude(void);
float baro_get_temp(void);

void sensor_init(void);


#include "mpu6050.h"
#include "hmc5883.h"
#include "ms5611.h"
