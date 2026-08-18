/*
 * lowpass.c
 *
 *  Created on: Aug 17, 2026
 *      Author: vepex
 */

#ifndef SRC_LOWPASS_C_
#define SRC_LOWPASS_C_

#include "lowpass.h"
#include <math.h>


void LowPass_Init(LowPass_t* filter , float cutoff_freq_hz , float sample_freq_hz){
	float dt = 1.0f/sample_freq_hz;
	float rc = 1.0f / (2.0f * 3.14159265f * cutoff_freq_hz);

	filter->alpha = dt / (rc + dt);

	filter->output = 0.0f;

	filter->initialized = 0;
}

float LowPass_Update(LowPass_t *filter, float input){
	if (!filter->initialized){
		filter->output = input;
		filter->initialized = 1;
	}
	else {
		filter->output += filter->alpha * (input - filter->output);
	}

	return filter->output;
}

#endif /* SRC_LOWPASS_C_ */
