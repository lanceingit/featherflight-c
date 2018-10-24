#include "mathlib.h"
#include <math.h>

float inv_sqrt(float x)
{
    float x_half = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (x_half * y * y));
    
    return y;
}

float constrain(float val, float min_val, float max_val)
{
	return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

float wrap_pi(float bearing)
{
	/* value is inf or NaN */
	if (!isfinite(bearing)) {
		return bearing;
	}

	int c = 0;

	while (bearing >= M_PI_F) {
		bearing -= M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	c = 0;

	while (bearing < -M_PI_F) {
		bearing += M_TWOPI_F;

		if (c++ > 3) {
			return NAN;
		}
	}

	return bearing;
}

float press2alt(float p)
{
	return 44330 * (1 - powf(((float)p / (float)1013.25),(1/5.255)));  //FIXME:
}
