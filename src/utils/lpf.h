#pragma once


struct lpf_s
{
    float cutoff_freq;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float delay_element_1;        // buffered sample -1
    float delay_element_2;        // buffered sample -2
};


void lpf_init(struct lpf_s* filter, float sample_freq, float cutoff_freq);


void lpf_set_cutoff_frequency(struct lpf_s* filter, float sample_freq, float cutoff_freq);

/**
 * Add a new raw value to the filter
 *
 * @return retrieve the filtered result
 */
float lpf_apply(struct lpf_s* filter, float sample);

/**
 * Return the cutoff frequency
 */
float lpf_get_cutoff_freq(struct lpf_s* filter);


/**
 * Reset the filter state to this value
 */
float lpf_reset(struct lpf_s* filter, float sample);

