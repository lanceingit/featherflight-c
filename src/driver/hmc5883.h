#pragma once

#include "sensor.h"

struct hmc5883_s 
{
    struct compass_s heir;
};

extern struct hmc5883_s hmc5883;

bool hmc5883_init(void);
void hmc5883_update(void);






