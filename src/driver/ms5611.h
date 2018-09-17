#pragma once

bool ms5611_init(void);
void ms5611_read(void);

float ms5611_get_press(void);
float ms5611_get_altitude(void);
float ms5611_get_temp(void);


