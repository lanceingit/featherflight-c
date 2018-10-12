#include <stdint.h>
#include "param.h"
#include "param_api.h"

struct vel_param_s {
	float vel_xy_i;
	float vel_z_d;
	float vel_z_i;
	float vel_xy_d;
	float vel_z_imax;
	float vel_z_imin;
	float vel_z_p;
	float vel_xy_p;
	float vel_xy_imax;
};

static inline void param_register_vel(void* param)
{
	vel_param = (struct vel_param_s*)param;
}

static inline float param_get_vel_xy_i(void)
{
    return vel_param->vel_xy_i;
}

static inline void param_set_vel_xy_i(float v)
{
    vel_param->vel_xy_i = v;
}

static inline float param_get_vel_z_d(void)
{
    return vel_param->vel_z_d;
}

static inline void param_set_vel_z_d(float v)
{
    vel_param->vel_z_d = v;
}

static inline float param_get_vel_z_i(void)
{
    return vel_param->vel_z_i;
}

static inline void param_set_vel_z_i(float v)
{
    vel_param->vel_z_i = v;
}

static inline float param_get_vel_xy_d(void)
{
    return vel_param->vel_xy_d;
}

static inline void param_set_vel_xy_d(float v)
{
    vel_param->vel_xy_d = v;
}

static inline float param_get_vel_z_imax(void)
{
    return vel_param->vel_z_imax;
}

static inline void param_set_vel_z_imax(float v)
{
    vel_param->vel_z_imax = v;
}

static inline float param_get_vel_z_imin(void)
{
    return vel_param->vel_z_imin;
}

static inline void param_set_vel_z_imin(float v)
{
    vel_param->vel_z_imin = v;
}

static inline float param_get_vel_z_p(void)
{
    return vel_param->vel_z_p;
}

static inline void param_set_vel_z_p(float v)
{
    vel_param->vel_z_p = v;
}

static inline float param_get_vel_xy_p(void)
{
    return vel_param->vel_xy_p;
}

static inline void param_set_vel_xy_p(float v)
{
    vel_param->vel_xy_p = v;
}

static inline float param_get_vel_xy_imax(void)
{
    return vel_param->vel_xy_imax;
}

static inline void param_set_vel_xy_imax(float v)
{
    vel_param->vel_xy_imax = v;
}

