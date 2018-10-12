#include <stdint.h>
#include "param.h"
#include "param_api.h"   

extern struct param_val param_list[] = {
	{"ATT_YAW_I", param_get_att_yaw_i, param_set_att_yaw_i},
	{"ATT_PITCH_D_W", param_get_att_pitch_d_w, param_set_att_pitch_d_w},
	{"ATT_PITCH_P", param_get_att_pitch_p, param_set_att_pitch_p},
	{"ATT_ROLL_D", param_get_att_roll_d, param_set_att_roll_d},
	{"ATT_ROLL_D_W", param_get_att_roll_d_w, param_set_att_roll_d_w},
	{"ATT_ROLL_I", param_get_att_roll_i, param_set_att_roll_i},
	{"ATT_PITCH_IMAX", param_get_att_pitch_imax, param_set_att_pitch_imax},
	{"ATT_YAW_D", param_get_att_yaw_d, param_set_att_yaw_d},
	{"ATT_PITCH_D", param_get_att_pitch_d, param_set_att_pitch_d},
	{"ATT_ROLL_P", param_get_att_roll_p, param_set_att_roll_p},
	{"ATT_ROLL_IMAX", param_get_att_roll_imax, param_set_att_roll_imax},
	{"ATT_YAW_P", param_get_att_yaw_p, param_set_att_yaw_p},
	{"ATT_PITCH_I", param_get_att_pitch_i, param_set_att_pitch_i},
	{"ATT_YAW_IMAX", param_get_att_yaw_imax, param_set_att_yaw_imax},

	{"VEL_XY_I", param_get_vel_xy_i, param_set_vel_xy_i},
	{"VEL_Z_D", param_get_vel_z_d, param_set_vel_z_d},
	{"VEL_Z_I", param_get_vel_z_i, param_set_vel_z_i},
	{"VEL_XY_D", param_get_vel_xy_d, param_set_vel_xy_d},
	{"VEL_Z_IMAX", param_get_vel_z_imax, param_set_vel_z_imax},
	{"VEL_Z_IMIN", param_get_vel_z_imin, param_set_vel_z_imin},
	{"VEL_Z_P", param_get_vel_z_p, param_set_vel_z_p},
	{"VEL_XY_P", param_get_vel_xy_p, param_set_vel_xy_p},
	{"VEL_XY_IMAX", param_get_vel_xy_imax, param_set_vel_xy_imax},

};

extern struct att_param_s* att_param;
extern struct vel_param_s* vel_param;
