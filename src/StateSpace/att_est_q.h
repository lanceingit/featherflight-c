#pragma once

#include "est.h"
#include "LowPassFilter2p.h"

struct att_est_q_s
{
	struct att_est_s heir;
    
	bool mag_decl_auto;
	float mag_decl;
    Vector mag_earth;
	float bias_max;
	float w_accel;
	float w_mag;
	float w_gyro_bias;    

    Vector rate;

	struct LowPassFilter2p	acc_filter_x;
	struct LowPassFilter2p	acc_filter_y;
	struct LowPassFilter2p	acc_filter_z;
	struct LowPassFilter2p	gyro_filter_x;
	struct LowPassFilter2p	gyro_filter_y;
	struct LowPassFilter2p	gyro_filter_z;
};

extern struct att_est_q_s att_est_q;

bool att_est_q_init(void);
bool att_est_q_run(float dt);
