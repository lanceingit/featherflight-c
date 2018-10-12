#include <stdint.h>
#include "param.h"
#include "param_api.h"

struct att_param_s {
	float pitch;
	float roll;
	float yaw;
};

struct att_param_s* att_param;

void param_register_att(void* param)
{
	att_param = (struct att_param_s*)param;
}

float param_get_att_pitch(void)
{
    return att_param->pitch;
}

void param_set_att_pitch(float v)
{
    att_param->pitch = v;
}

float param_get_att_roll(void)
{
    return att_param->roll;
}

void param_set_att_roll(float v)
{
    att_param->roll = v;
}

float param_get_att_yaw(void)
{
    return att_param->yaw;
}

void param_set_att_yaw(float v)
{
    att_param->yaw = v;
}




