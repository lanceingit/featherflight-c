#include <stdint.h>
#include "param.h"
#include "param_api.h"

struct att_param_s {
	float att_yaw_i;
	float att_pitch_d_w;
	float att_pitch_p;
	float att_roll_d;
	float att_roll_d_w;
	float att_roll_i;
	float att_pitch_imax;
	float att_yaw_d;
	float att_pitch_d;
	float att_roll_p;
	float att_roll_imax;
	float att_yaw_p;
	float att_pitch_i;
	float att_yaw_imax;
};

static inline void param_register_att(void* param)
{
	att_param = (struct att_param_s*)param;
}

static inline float param_get_att_yaw_i(void)
{
    return att_param->att_yaw_i;
}

static inline void param_set_att_yaw_i(float v)
{
    att_param->att_yaw_i = v;
}

static inline float param_get_att_pitch_d_w(void)
{
    return att_param->att_pitch_d_w;
}

static inline void param_set_att_pitch_d_w(float v)
{
    att_param->att_pitch_d_w = v;
}

static inline float param_get_att_pitch_p(void)
{
    return att_param->att_pitch_p;
}

static inline void param_set_att_pitch_p(float v)
{
    att_param->att_pitch_p = v;
}

static inline float param_get_att_roll_d(void)
{
    return att_param->att_roll_d;
}

static inline void param_set_att_roll_d(float v)
{
    att_param->att_roll_d = v;
}

static inline float param_get_att_roll_d_w(void)
{
    return att_param->att_roll_d_w;
}

static inline void param_set_att_roll_d_w(float v)
{
    att_param->att_roll_d_w = v;
}

static inline float param_get_att_roll_i(void)
{
    return att_param->att_roll_i;
}

static inline void param_set_att_roll_i(float v)
{
    att_param->att_roll_i = v;
}

static inline float param_get_att_pitch_imax(void)
{
    return att_param->att_pitch_imax;
}

static inline void param_set_att_pitch_imax(float v)
{
    att_param->att_pitch_imax = v;
}

static inline float param_get_att_yaw_d(void)
{
    return att_param->att_yaw_d;
}

static inline void param_set_att_yaw_d(float v)
{
    att_param->att_yaw_d = v;
}

static inline float param_get_att_pitch_d(void)
{
    return att_param->att_pitch_d;
}

static inline void param_set_att_pitch_d(float v)
{
    att_param->att_pitch_d = v;
}

static inline float param_get_att_roll_p(void)
{
    return att_param->att_roll_p;
}

static inline void param_set_att_roll_p(float v)
{
    att_param->att_roll_p = v;
}

static inline float param_get_att_roll_imax(void)
{
    return att_param->att_roll_imax;
}

static inline void param_set_att_roll_imax(float v)
{
    att_param->att_roll_imax = v;
}

static inline float param_get_att_yaw_p(void)
{
    return att_param->att_yaw_p;
}

static inline void param_set_att_yaw_p(float v)
{
    att_param->att_yaw_p = v;
}

static inline float param_get_att_pitch_i(void)
{
    return att_param->att_pitch_i;
}

static inline void param_set_att_pitch_i(float v)
{
    att_param->att_pitch_i = v;
}

static inline float param_get_att_yaw_imax(void)
{
    return att_param->att_yaw_imax;
}

static inline void param_set_att_yaw_imax(float v)
{
    att_param->att_yaw_imax = v;
}

