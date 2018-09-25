#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "sensor.h"
#include "att_est_q.h"
#include "timer.h"
#include "vector.h"
#include "quaternion.h"
#include "matrix.h"
#include "LowPassFilter2p.h"
#include "geo.h"
#include "mathlib.h"

#if 0
#define LINK_DEBUG(a) _link->send_text(a)
#else
#define LINK_DEBUG(a)
#endif

struct att_est_q_s
{
    bool inited;
    bool use_compass;
	bool mag_decl_auto;
	float mag_decl;
    Vector mag_earth;
	uint64_t last_time;
	float dt_max;
	float bias_max;
	float w_accel;
	float w_mag;
	float w_gyro_bias;    

    Vector acc;
    Vector gyro;
    Vector mag;
    Vector gyro_bias;

    Vector rate;
    Quaternion q;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float roll;
    float pitch;
    float yaw;  

	struct LowPassFilter2p	acc_filter_x;
	struct LowPassFilter2p	acc_filter_y;
	struct LowPassFilter2p	acc_filter_z;
	struct LowPassFilter2p	gyro_filter_x;
	struct LowPassFilter2p	gyro_filter_y;
	struct LowPassFilter2p	gyro_filter_z;
} att;

static struct att_est_q_s* this=&att;

static bool att_est_q_update(float dt);

bool att_est_q_init(void)
{
    inertial_sensor_get_acc(&this->acc);

	Vector k = vector_normalized(vector_reverse(this->acc));

	if (vector_length(this->acc) < 0.01f || vector_length(this->acc) > 12) {
		LINK_DEBUG("init: degenerate accel!");
	}

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
    Vector i = {1, 0, 0};

	if(this->use_compass)
    {
        inertial_sensor_get_acc(&this->mag);
        //esprintf(buf, "mag0:%.3f 1:%.3f 2:%.3f", (double)_mag(0),(double)_mag(1),(double)_mag(2));
        LINK_DEBUG(buf);
        if (vector_length(this->mag) < 0.01f) {
            LINK_DEBUG("init: degenerate mag!");
        }
        
        i = vector_normalized(vector_sub(this->mag, vector_mul(k, vector_scalar(this->mag, k))));
    }
    


	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector j = vector_cross(k, i);;
    
	// Fill rotation matrix
	Matrix R={3,3};
	MAT(R, 0, 0) = i.x;
	MAT(R, 0, 1) = i.y;
	MAT(R, 0, 2) = i.z;
	MAT(R, 1, 0) = j.x;
	MAT(R, 1, 1) = j.y;
	MAT(R, 1, 2) = j.z;
	MAT(R, 2, 0) = k.x;
	MAT(R, 2, 1) = k.y;
	MAT(R, 2, 2) = k.z;

	// Convert to quaternion
	this->q = quaternion_from_dcm(R);

	// Compensate for magnetic declination
	Quaternion decl_rotation = quaternion_from_yaw(this->mag_decl);
    this->q = quaternion_normalize(quaternion_mul(decl_rotation, this->q));

	if (isfinite(this->q.w) && isfinite(this->q.x) &&
			isfinite(this->q.y) && isfinite(this->q.z) &&
	    quaternion_length(this->q) > 0.95f && quaternion_length(this->q) < 1.05f) {

		this->inited = true;
	}
	else
	{
		this->inited = false;
		LINK_DEBUG("q init definite");
	}

	return this->inited;
}

void att_est_q_run(void)
{
	if(inertial_sensor_ready())
	{
		this->gyro.x = lowPassFilter2p_apply(&this->gyro_filter_x, inertial_sensor_get_gyro_x());
		this->gyro.y = lowPassFilter2p_apply(&this->gyro_filter_y, inertial_sensor_get_gyro_y());
		this->gyro.z = lowPassFilter2p_apply(&this->gyro_filter_z, inertial_sensor_get_gyro_z());

		this->acc.x = lowPassFilter2p_apply(&this->acc_filter_x, inertial_sensor_get_acc_x());
		this->acc.y = lowPassFilter2p_apply(&this->acc_filter_y, inertial_sensor_get_acc_y());
		this->acc.z = lowPassFilter2p_apply(&this->acc_filter_z, inertial_sensor_get_acc_z());

		if (vector_length(this->acc) < 0.01f) {
			LINK_DEBUG("WARNING: degenerate accel!");
			return;
		}
	}
	else
	{
		return;
	}

	if(this->use_compass)
	{
        inertial_sensor_get_acc(&this->mag);

		if (vector_length(this->mag) < 0.01f) {
			LINK_DEBUG("WARNING: degenerate mag!");
			return;
		}
    }

	/* time from previous iteration */
	uint64_t now = Timer_getTime();
	float dt = (this->last_time > 0) ? ((now  - this->last_time) / 1000000.0f) : 0.00001f;
	this->last_time = now;

	if (dt > this->dt_max) {
		dt = this->dt_max;
	}

	if (!att_est_q_update(dt)) {
		return;
	}

	{
		Vector euler;
        //        quaternion_to_euler(this->q, euler);    //TODO:

		this->roll_rate = this->rate.x;
		this->pitch_rate = this->rate.y;
		this->yaw_rate = this->rate.z;

	    this->roll = euler.x;
	    this->pitch = euler.y;
	    this->yaw = euler.z;
	}

}

bool att_est_q_update(float dt)
{
	if (!this->inited) {

		return att_est_q_init();
	}

	Quaternion q_last = this->q;

	// Angular rate of correction
	Vector corr;
	float spinRate = vector_length(this->gyro);

	if (this->use_compass) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
        this->mag_earth = quaternion_conjugate(this->q, this->mag);
		float mag_err = _wrap_pi(atan2f(this->mag_earth.y, this->mag_earth.x) - this->mag_decl);

		// Project magnetometer correction to body frame
        corr = vector_add(corr, vector_mul(quaternion_conjugate_inversed(this->q, vector_set(0.0f, 0.0f, -mag_err)), this->w_mag));
	}

	this->q = quaternion_normalize(this->q);


	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector<3> k = _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector k;
	k = vector_set(2.0f * (this->q.x * this->q.z - this->q.w * this->q.y),
		           2.0f * (this->q.y * this->q.z + this->q.w * this->q.x),
		           (this->q.w * this->q.w - this->q.x * this->q.x - this->q.y * this->q.y + this->q.z * this->q.z)
			   );



    corr = vector_add(corr, vector_mul(vector_cross(k, vector_normalized(this->acc)), this->w_accel));
	//_corr_acc = corr;

	// Gyro bias estimation
	if (spinRate < 0.175f) {
        this->gyro_bias = vector_add(this->gyro_bias, vector_mul(corr, (this->w_gyro_bias * dt)));

        this->gyro_bias.x = constrain(this->gyro_bias.x, -this->bias_max, this->bias_max);
        this->gyro_bias.y = constrain(this->gyro_bias.y, -this->bias_max, this->bias_max);
        this->gyro_bias.z = constrain(this->gyro_bias.z, -this->bias_max, this->bias_max);
	}

    this->rate = vector_add(this->gyro, this->gyro_bias);

	// Feed forward gyro
    corr = vector_add(corr, this->rate);

	// Apply correction to state
	this->q = quaternion_normalize(quaternion_add(this->q, quaternion_scaler(quaternion_derivative(this->q, corr), dt)));

	if (!(isfinite(this->q.w) && isfinite(this->q.x) &&
			isfinite(this->q.y) && isfinite(this->q.z))) {
		// Reset quaternion to last good state
		this->q = q_last;
		this->rate = vector_set(0,0,0);
        this->gyro_bias = vector_set(0,0,0);
		LINK_DEBUG("q definite");
		return false;
	}

	return true;
}

float att_get_roll(void)
{
    return this->roll;
}

float att_get_pitch(void)
{
    return this->pitch;
}

float att_get_yaw(void)
{
    return this->yaw;
}
