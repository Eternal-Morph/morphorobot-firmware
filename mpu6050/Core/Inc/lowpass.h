/*
 * lowpass.h
 *
 *  Created on: Aug 17, 2026
 *      Author: vepex
 */

#ifndef INC_LOWPASS_H_
#define INC_LOWPASS_H_

#include "main.h"

typedef struct{
	float alpha;
	float output;
	uint8_t initialized;

}LowPass_t;

void LowPass_Init(LowPass_t* filter , float cutoff_freq_hz , float sample_freq_hz);

float LowPass_Update(LowPass_t *filter, float input);

#endif /* INC_LOWPASS_H_ */
