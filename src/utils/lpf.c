#include <math.h>
#include "mathlib.h"
#include "lpf.h"

void lpf_init(struct lpf_s* filter, float sample_freq, float cutoff_freq)
{
	filter->cutoff_freq=cutoff_freq;
	filter->a1=0.0f;
	filter->a2=0.0f;
	filter->b0=0.0f;
	filter->b1=0.0f;
	filter->b2=0.0f;
	filter->delay_element_1=0.0f;
	filter->delay_element_2=0.0f;
        // set initial parameters
	lpf_set_cutoff_frequency(filter, sample_freq, cutoff_freq);
}



void lpf_set_cutoff_frequency(struct lpf_s* filter, float sample_freq, float cutoff_freq)
{
	filter->cutoff_freq = cutoff_freq;
    if (filter->cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }
    float fr = sample_freq/filter->cutoff_freq;
    float ohm = tanf(M_PI_F/fr);
    float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
    filter->b0 = ohm*ohm/c;
    filter->b1 = 2.0f*filter->b0;
    filter->b2 = filter->b0;
    filter->a1 = 2.0f*(ohm*ohm-1.0f)/c;
    filter->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
}

float lpf_apply(struct lpf_s* filter, float sample)
{
    if (filter->cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do the filtering
    float delay_element_0 = sample - filter->delay_element_1 * filter->a1 - filter->delay_element_2 * filter->a2;
    if (!isfinite(delay_element_0)) {
        // don't allow bad values to propagate via the filter
        delay_element_0 = sample;
    }
    float output = delay_element_0 * filter->b0 + filter->delay_element_1 * filter->b1 + filter->delay_element_2 * filter->b2;
    
    filter->delay_element_2 = filter->delay_element_1;
    filter->delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

float lpf_reset(struct lpf_s* filter, float sample) {
	float dval = sample / (filter->b0 + filter->b1 + filter->b2);
	filter->delay_element_1 = dval;
	filter->delay_element_2 = dval;
    return lpf_apply(filter, sample);
}

float lpf_get_cutoff_freq(struct lpf_s* filter)
{
	return filter->cutoff_freq;
}


