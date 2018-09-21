#pragma once

#include "sensor.h"

struct hmc5883_s 
{
    struct compass_s heir;
};

extern struct hmc5883_s hmc5883;

bool hmc5883_init(void);
void hmc5883_read(void);

float hmc5883_get_mag_x();
float hmc5883_get_mag_y();
float hmc5883_get_mag_z();
void hmc5883_set_mag(float *m);
void hmc5883_set_mag_x(float m);
void hmc5883_set_mag_y(float m);
void hmc5883_set_mag_z(float m);





