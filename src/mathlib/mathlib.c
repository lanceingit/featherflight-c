#include "board.h"
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



void variance_collect(struct variance_s* v, float val)
{
	v->cnt++;
	v->sum += val;
    v->sum_x2 += POW2(val);	
}

float variance_cal(struct variance_s* v)
{
	float s2;
	float s3;
	float avg;

	avg = v->sum / v->cnt;
	s2 = 2 * avg * v->sum;
    s3 = v->cnt * POW2(avg);
    return (v->sum_x2 - s2 + s3)/v->cnt; 
}

struct variance_s
{
	float sum;
	uint8_t size;
	struct fifo_s fifo;
	uint8_t data[100];
};

float variance_create(struct variance_s* v, uint8_t size)
{
	v->size = size;
	for(uint8_t i=0; i<v->size; i++) {
		v->data[i] = 0.0f;
	}
	fifo_create(&v->data, v->data, v->size);
}

float variance_cal(struct variance_s* v, float val)
{
	// float 

	// if(fifo_get_count(v) < v->size) {
	// 	v->sum += val;
	// } else {
	// 	fifo_read();
	// 	v->sum = v->sum - 
	// }
}

