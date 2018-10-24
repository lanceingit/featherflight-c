#include "board.h"

#include "est.h"
#include "timer.h"
#include "sensor.h"

#define LINK_DEBUG  printf

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
	if(inertial_sensor_ready(0)) {
		est_att->gyro.x = inertial_sensor_get_gyro_x(0);
		est_att->gyro.y = inertial_sensor_get_gyro_y(0);
		est_att->gyro.z = inertial_sensor_get_gyro_z(0);

		est_att->acc.x = inertial_sensor_get_acc_x(0);
		est_att->acc.y = inertial_sensor_get_acc_y(0);
		est_att->acc.z = inertial_sensor_get_acc_z(0);

		if (vector_length(est_att->acc) < 0.01f) {
			LINK_DEBUG("WARNING: degenerate accel!");
			return;
		}
	} else {
		return;
	}

	if(est_att->use_compass) {
        compass_get_mag(0, &est_att->mag);

		if (vector_length(est_att->mag) < 0.01f) {
			LINK_DEBUG("WARNING: degenerate mag!");
			return;
		}
    }

	/* time from previous iteration */
	times_t now = timer_now();
	float dt = (est_att->last_time > 0) ? ((now  - est_att->last_time) / 1000000.0f) : 0.00001f;
	est_att->last_time = now;

	if (dt > est_att->dt_max) {
		dt = est_att->dt_max;
	}

	if (!est_att->heir.run(dt)) {
		return;
	}

    Vector euler = quaternion_to_euler(est_att->q);  

    est_att->roll_rate = inertial_sensor_get_gyro_x(0) + est_att->gyro_bias.x;
    est_att->pitch_rate = inertial_sensor_get_gyro_y(0) + est_att->gyro_bias.y;
    est_att->yaw_rate = inertial_sensor_get_gyro_z(0) + est_att->gyro_bias.z;

    est_att->roll = euler.x;
    est_att->pitch = euler.y;
    est_att->yaw = euler.z;    
}
