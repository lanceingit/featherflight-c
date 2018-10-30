#include "board.h"

#include "est.h"
#include "timer.h"
#include "sensor.h"
#include "debug.h"
#include "mathlib.h"

static struct att_est_s* est_att;

void att_est_register(struct att_est_s* att)
{
    est_att = att;
}

float att_get_roll(void)
{
    return est_att->roll;
}

float att_get_pitch(void)
{
    return est_att->pitch;
}

float att_get_yaw(void)
{
    return est_att->yaw;
}

float att_get_roll_rate(void)
{
    return est_att->roll_rate;
}

float att_get_pitch_rate(void)
{
    return est_att->pitch_rate;
}

float att_get_yaw_rate(void)
{
    return est_att->yaw_rate;
}

bool att_valid(void)
{
	return est_att->valid;  
}

void att_get_dcm(Dcm r)
{
	quaternion_to_dcm(est_att->q, r);
}

void est_init(void)
{
    est_att->use_compass = false;
	est_att->last_time = 0;
	est_att->dt_max = 0.02f;
	est_att->inited = false;    
    
    est_att->heir.init();   
}

void est_att_run(void)
{
	if(imu_ready(0)) {
		est_att->gyro.x = imu_get_gyro_x(0);
		est_att->gyro.y = imu_get_gyro_y(0);
		est_att->gyro.z = imu_get_gyro_z(0);

		est_att->acc.x = imu_get_acc_x(0);
		est_att->acc.y = imu_get_acc_y(0);
		est_att->acc.z = imu_get_acc_z(0);

		if (vector_length(est_att->acc) < 0.01f) {
			PRINT("WARNING: degenerate accel!\n");
			return;
		}
	} else {
		return;
	}

	if(est_att->use_compass) {
        compass_get_mag(0, &est_att->mag);

		if (vector_length(est_att->mag) < 0.01f) {
			PRINT("WARNING: degenerate mag!\n");
			return;
		}
    }

	float dt = timer_get_dt(&est_att->last_time, est_att->dt_max, 0.00001f);

	if (!est_att->heir.run(dt)) {
		return;
	}

    Vector euler = quaternion_to_euler(est_att->q);  

    est_att->roll_rate =  (imu_get_gyro_x(0) + est_att->gyro_bias.x)*M_RAD_TO_DEG;
    est_att->pitch_rate = (imu_get_gyro_y(0) + est_att->gyro_bias.y)*M_RAD_TO_DEG;
    est_att->yaw_rate =   (imu_get_gyro_z(0) + est_att->gyro_bias.z)*M_RAD_TO_DEG;

    est_att->roll = euler.x*M_RAD_TO_DEG;
    est_att->pitch = euler.y*M_RAD_TO_DEG;
    est_att->yaw = euler.z*M_RAD_TO_DEG;    
}
