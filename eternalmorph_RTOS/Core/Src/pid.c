/*
 * pid.c
 *
 *  Created on: Aug 16, 2026
 *      Author: vepex
 */


#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float lim_min, float lim_max, float lim_max_int){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->lim_min = lim_min;
	pid->lim_max = lim_max;
	pid->lim_max_int = lim_max_int;
	pid->lim_min_int = -lim_max_int;
	PID_Reset(pid);
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt){
	if (dt <= 0.0f){
		return pid->out;
	}
	else{
	float error = (setpoint-measurement);

	float p_term = pid->Kp * error;

	pid->integral += error*dt;

	if(pid->integral > pid->lim_max_int){
		pid->integral = pid->lim_max_int;
	}
	if(pid->integral < pid->lim_min_int){
		pid->integral = pid->lim_min_int;
	}

	float i_term = pid->Ki*pid->integral;

	float deriv = (measurement - pid->prev_measurement)/dt;

	float d_term = deriv * (-pid->Kd);

	pid->out = p_term + i_term + d_term;

	if(pid->out > pid->lim_max){
		pid->out = pid->lim_max;
	}
	if(pid->out < pid->lim_min){
		pid->out = pid->lim_min;
	}

	pid->prev_measurement = measurement;

	return pid->out;

	}
}

void PID_Reset(PID_t *pid){
	pid->integral = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->prev_d = 0.0f;
	pid->out = 0.0f;
}
