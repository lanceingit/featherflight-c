#pragma once

#include <stdint.h>
#include "param.h"
#include "param_api.h"

struct att_param_s {
	float att_bias_max;
	float att_w_accel;
	float att_w_mag;
	float att_w_gyro_bias;
};

extern struct att_param_s* att_param;
static inline void param_register_att(void* param)
{
	att_param = (struct att_param_s*)param;
}

static inline float param_get_att_bias_max(void)
{
    return att_param->att_bias_max;
}

static inline void param_set_att_bias_max(float v)
{
    att_param->att_bias_max = v;
}

static inline float param_get_att_w_accel(void)
{
    return att_param->att_w_accel;
}

static inline void param_set_att_w_accel(float v)
{
    att_param->att_w_accel = v;
}

static inline float param_get_att_w_mag(void)
{
    return att_param->att_w_mag;
}

static inline void param_set_att_w_mag(float v)
{
    att_param->att_w_mag = v;
}

static inline float param_get_att_w_gyro_bias(void)
{
    return att_param->att_w_gyro_bias;
}

static inline void param_set_att_w_gyro_bias(float v)
{
    att_param->att_w_gyro_bias = v;
}

