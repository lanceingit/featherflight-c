#pragma once


struct LowPassFilter2p
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


void lowPassFilter2p_init(struct LowPassFilter2p* filter, float sample_freq, float cutoff_freq);


void lowPassFilter2p_set_cutoff_frequency(struct LowPassFilter2p* filter, float sample_freq, float cutoff_freq);

/**
 * Add a new raw value to the filter
 *
 * @return retrieve the filtered result
 */
float lowPassFilter2p_apply(struct LowPassFilter2p* filter, float sample);

/**
 * Return the cutoff frequency
 */
float lowPassFilter2p_get_cutoff_freq(struct LowPassFilter2p* filter);


/**
 * Reset the filter state to this value
 */
float lowPassFilter2p_reset(struct LowPassFilter2p* filter, float sample);

