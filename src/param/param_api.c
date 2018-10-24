#include <stdint.h>
#include "param.h"
#include "param_api.h"   

struct param_val param_list[] = {
	{"ATT_BIAS_MAX", param_get_att_bias_max, param_set_att_bias_max},
	{"ATT_W_ACCEL", param_get_att_w_accel, param_set_att_w_accel},
	{"ATT_W_MAG", param_get_att_w_mag, param_set_att_w_mag},
	{"ATT_W_GYRO_BIAS", param_get_att_w_gyro_bias, param_set_att_w_gyro_bias},

	{"ATTC_RATE_YAW_D", param_get_attc_rate_yaw_d, param_set_attc_rate_yaw_d},
	{"ATTC_RATE_PITCH_D", param_get_attc_rate_pitch_d, param_set_attc_rate_pitch_d},
	{"ATTC_RATE_RP_D_WEIGHT", param_get_attc_rate_rp_d_weight, param_set_attc_rate_rp_d_weight},
	{"ATTC_RATE_YAW_I_LIMIT", param_get_attc_rate_yaw_i_limit, param_set_attc_rate_yaw_i_limit},
	{"ATTC_RATE_YAW_D_WEIGHT", param_get_attc_rate_yaw_d_weight, param_set_attc_rate_yaw_d_weight},
	{"ATTC_RATE_YAW_P", param_get_attc_rate_yaw_p, param_set_attc_rate_yaw_p},
	{"ATTC_RATE_PITCH_P", param_get_attc_rate_pitch_p, param_set_attc_rate_pitch_p},
	{"ATTC_ATT_YAW_OUT_LIMIT", param_get_attc_att_yaw_out_limit, param_set_attc_att_yaw_out_limit},
	{"ATTC_RATE_ROLL_P", param_get_attc_rate_roll_p, param_set_attc_rate_roll_p},
	{"ATTC_RATE_RP_OUT_LIMIT", param_get_attc_rate_rp_out_limit, param_set_attc_rate_rp_out_limit},
	{"ATTC_RATE_YAW_OUT_LIMIT", param_get_attc_rate_yaw_out_limit, param_set_attc_rate_yaw_out_limit},
	{"ATTC_ATT_ROLL_P", param_get_attc_att_roll_p, param_set_attc_att_roll_p},
	{"ATTC_RATE_PITCH_I", param_get_attc_rate_pitch_i, param_set_attc_rate_pitch_i},
	{"ATTC_RP_LIMIT", param_get_attc_rp_limit, param_set_attc_rp_limit},
	{"ATTC_RATE_ROLL_I", param_get_attc_rate_roll_i, param_set_attc_rate_roll_i},
	{"ATTC_ATT_YAW_P", param_get_attc_att_yaw_p, param_set_attc_att_yaw_p},
	{"ATTC_RATE_ROLL_D", param_get_attc_rate_roll_d, param_set_attc_rate_roll_d},
	{"ATTC_ATT_RP_OUT_LIMIT", param_get_attc_att_rp_out_limit, param_set_attc_att_rp_out_limit},
	{"ATTC_RATE_YAW_I", param_get_attc_rate_yaw_i, param_set_attc_rate_yaw_i},
	{"ATTC_RATE_RP_I_LIMIT", param_get_attc_rate_rp_i_limit, param_set_attc_rate_rp_i_limit},
	{"ATTC_ATT_PITCH_P", param_get_attc_att_pitch_p, param_set_attc_att_pitch_p},

};

struct att_param_s* att_param;
struct attc_param_s* attc_param;
