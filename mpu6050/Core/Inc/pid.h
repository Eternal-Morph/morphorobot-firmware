/*
 * pid.h
 *
 *  Created on: Aug 16, 2026
 *      Author: vepex
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct{

	float Kp;
	float Ki;
	float Kd;

	float integral;
	float prev_measurement;
	float prev_d;

	float lim_min_int, lim_max_int;
	float lim_min, lim_max;

	float out;
}PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float lim_min, float lim_max, float lim_max_int);

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

void PID_Reset(PID_t *pid);


#endif /* INC_PID_H_ */
