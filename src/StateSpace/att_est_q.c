#include <stdbool.h>
#include "att_est_q.h"
#include "timer.h"
#include "vector.h"
#include "quaternion.h"
#include "LowPassFilter2p.h"





#if 0
#define LINK_DEBUG(a) _link->send_text(a)
#else
#define LINK_DEBUG(a)
#endif

Att_Est_Q::Att_Est_Q(SENSORS* sensor) :
	_lp_accel_x(625.0f, 30.0f),
	_lp_accel_y(625.0f, 30.0f),
	_lp_accel_z(625.0f, 30.0f),
	_lp_gyro_x(625.0f, 30.0f),
	_lp_gyro_y(625.0f, 30.0f),
	_lp_gyro_z(625.0f, 30.0f),
	_use_compass(false),
	_mag_decl_auto(true),
	_mag_decl(0.0f),
	_last_time(0),
	_dt_max(0.02f),
	_bias_max(10.05f),
	_w_accel(0.2f),
	_w_mag(0.1f),
	_w_gyro_bias(0.1f),
//	_link(link_mavlink),
    _sensors(sensor),
	_inited(false)
{
}


static float att_r[3][3];
static float q[4];


struct att_est_q_s
{
    bool inited;
    bool use_compass;
	bool mag_decl_auto;
	float mag_decl;
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

static struct att_est_q_s* this;

bool att_est_q_init(void)
{
    inertial_sensor_get_acc(this->acc);

	Vector k;
    vector_copy(k, vector_reverse_cal(this->acc));
    vector_normalized(k);

	if (vector_length(this->acc) < 0.01f || vector_length(this->acc) > 12) {
		LINK_DEBUG("init: degenerate accel!");
	}

	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
    Vector i;
    vector_set(i, 1, 0 ,0);

	if(_use_compass)
    {
        inertial_sensor_get_acc(this->mag);
        //esprintf(buf, "mag0:%.3f 1:%.3f 2:%.3f", (double)_mag(0),(double)_mag(1),(double)_mag(2));
        LINK_DEBUG(buf);
        if (vector_length(this->mag) < 0.01f) {
            LINK_DEBUG("init: degenerate mag!");
        }
        
        vector_sub(i, this->mag, vector_mul_cal(k, vector_scalar_cal(this->mag, k)));

        vector_normalized(i);
    }
    


	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector j;
    vector_cross(j, k, i);

	// Fill rotation matrix
	Matrix<3, 3> R;
	R.set_row(0, i);
	R.set_row(1, j);
	R.set_row(2, k);

	// Convert to quaternion
	_q.from_dcm(R);

	// Compensate for magnetic declination
	Quaternion decl_rotation;
	quaternion_from_yaw(decl_rotation, _mag_decl);
    quaternion_mul(this->q, decl_rotation, this->q);

	quaternion_normalize(this->q);

	if (isfinite(this->q[0]) && isfinite(this->q[1]) &&
			isfinite(this->q[2]) && isfinite(this->q[3]) &&
	    quaternion_length(this->q) > 0.95f && quaternion_length(this->q) < 1.05f) {

		this->inited = true;
	}
	else
	{
		this->inited = false;
		LINK_DEBUG("q init definite");
	}

	return _inited;
}

void att_est_q_run()
{
	if(inertial_sensor_ready())
	{
		this->gyro[0] = lowPassFilter2p_apply(&this->gyro_filter_x, inertial_sensor_get_gyro_x());
		this->gyro[1] = lowPassFilter2p_apply(&this->gyro_filter_y, inertial_sensor_get_gyro_y());
		this->gyro[2] = lowPassFilter2p_apply(&this->gyro_filter_z, inertial_sensor_get_gyro_z());

		this->acc[0] = lowPassFilter2p_apply(&this->acc_filter_x, inertial_sensor_get_acc_x());
		this->acc[1] = lowPassFilter2p_apply(&this->acc_filter_y, inertial_sensor_get_acc_y());
		this->acc[2] = lowPassFilter2p_apply(&this->acc_filter_z, inertial_sensor_get_acc_z());

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
        inertial_sensor_get_acc(this->mag);

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

	if (!update(dt)) {
		return;
	}

	{
		Vector euler;
        quaternion_to_euler(this->q, euler);

		this->roll_rate = this->rate[0];
		this->pitch_rate = this->rate[1];
		this->yaw_rate = this->rate[2];

	    this->roll = euler[0];
	    this->pitch = euler[1];
	    this->yaw = euler[2];
	}

}

bool Att_Est_Q::update(float dt)
{
	if (!_inited) {

		return init();
	}

	Quaternion q_last;
    quaternion_copy(q_last, this->q);

	// Angular rate of correction
	Vector corr;
	float spinRate = vector_length(this->gyro);

	if (_use_compass) {
		// Magnetometer correction
		// Project mag field vector to global frame and extract XY component
        quaternion_conjugate(mag_earth, this->q, this->mag);
		float mag_err = _wrap_pi(atan2f(mag_earth[1], mag_earth[0]) - this->mag_decl);

		// Project magnetometer correction to body frame
        vector_add(corr, corr, vector_scalar_cal(quaternion_conjugate_inversed_cal(this->q, vector_get(0.0f, 0.0f, -mag_err)), this->w_mag));
	}

	quaternion_normalize(this->q);


	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	// Vector<3> k = _q.conjugate_inversed(Vector<3>(0.0f, 0.0f, 1.0f));
	// Optimized version with dropped zeros
	Vector k;
    vector_set(k, 2.0f * (this->q[1] * this->q[3] - this->q[0] * this->q[2]),
		          2.0f * (this->q[2] * this->q[3] + this->q[0] * this->q[1]),
		          (this->q[0] * this->q[0] - this->q[1] * this->q[1] - this->q[2] * this->q[2] + this->q[3] * this->q[3]));



    vector_add(corr, corr, vector_mul_cal(vector_cross_cal(k, vector_normalized_cal(acc)), this->w_accel));
	//_corr_acc = corr;

	// Gyro bias estimation
	if (spinRate < 0.175f) {
        vector_add(this->gyro_bias, this->gyro_bias, vector_scalar_cal(corr, (_w_gyro_bias * dt)));

		for (int i = 0; i < 3; i++) {
			this->gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
		}

	}

    vector_add(this->rate, this->gyro, this->gyro_bias);

	// Feed forward gyro
    vector_add(corr, corr, this->rate);

	// Apply correction to state
	_q += _q.derivative(corr) * dt;

	// Normalize quaternion
	_q.normalize();

	if (!(isfinite(this->q[0]) && isfinite(this->q[1]) &&
			isfinite(this->q[2]) && isfinite(this->q[3]))) {
		// Reset quaternion to last good state
		_q = q_last;
		_rates.zero();
        vector_zero(this->gyro_bias);
		LINK_DEBUG("q definite");
		return false;
	}

	return true;
}
