#include "att_est_q.h"
#include "timer.h"
#include "geo.h"
#include <cmath>
#include <string>





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


static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                	      float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

    // Calculate general spin rate (rad/s)
    float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    float ex = 0, ey = 0, ez = 0;

    // Use measured magnetic field vector
    float recipMagNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipMagNorm > 0.01f) {
        // Normalise magnetometer measurement
        recipMagNorm = invSqrt(recipMagNorm);
        mx *= recipMagNorm;
        my *= recipMagNorm;
        mz *= recipMagNorm;

        // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
        // This way magnetic field will only affect heading and wont mess roll/pitch angles

        // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
        // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        float hx = att_r[0][0] * mx + att_r[0][1] * my + att_r[0][2] * mz;
        float hy = att_r[1][0] * mx + att_r[1][1] * my + att_r[1][2] * mz;
        float bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        float ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex += att_r[2][0] * ez_ef;
        ey += att_r[2][1] * ez_ef;
        ez += att_r[2][2] * ez_ef;
    }

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * att_r[2][2] - az * att_r[2][1]);
        ey += (az * att_r[2][0] - ax * att_r[2][2]);
        ez += (ax * att_r[2][1] - ay * att_r[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;
}
