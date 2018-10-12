#include <stdint.h>
#include "param.h"
#include "param_api.h"   

struct param_val param_list[] = {
	{"ATT_BIAS_MAX", param_get_att_bias_max, param_set_att_bias_max},
	{"ATT_W_ACCEL", param_get_att_w_accel, param_set_att_w_accel},
	{"ATT_W_MAG", param_get_att_w_mag, param_set_att_w_mag},
	{"ATT_W_GYRO_BIAS", param_get_att_w_gyro_bias, param_set_att_w_gyro_bias},

};

struct att_param_s* att_param;
