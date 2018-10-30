#pragma once

struct stick_s
{
    float roll;
    float pitch;
    float yaw;
    float thrust;
};

bool system_armed(void);

float commander_get_roll(void);
float commander_get_pitch(void);
float commander_get_yaw(void);
float commander_get_thrust(void);
float commander_get_yaw_rate(void);

void commander_set_roll(uint8_t ch, float v);
void commander_set_pitch(uint8_t ch, float v);
void commander_set_yaw(uint8_t ch, float v);
void commander_set_thrust(uint8_t ch, float v);

void commander_init(void);
void commander_update(void);

