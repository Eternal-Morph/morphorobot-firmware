/*
 * mixer.c
 *
 *  Created on: Aug 17, 2026
 *      Author: vepex
 */

#include "mixer.h"

void Mixer_Init(Motor_Output_t* motors){
	motors->m1 = 1000;
	motors->m2 = 1000;
	motors->m3 = 1000;
	motors->m4 = 1000;
	motors->left_wheel = 1000;
	motors->right_wheel = 1000;
}

static uint16_t clamp_pwm(float val) {
    if (val < 1000.0f) return 1000;
    if (val > 2000.0f) return 2000;
    return (uint16_t)val;
}

void Mixer_Update(Motor_Output_t* motors, RobotModel_t mode, float throttle, float roll, float pitch, float yaw, uint8_t is_armed){

	if(is_armed == 0){
		Mixer_Init(motors);
		return;
	}

	if(mode == MODE_AIR){
		float m1 = throttle - roll - pitch - yaw;
		float m2 = throttle - roll + pitch + yaw;
		float m3 = throttle + roll + pitch - yaw;
		float m4 = throttle + roll - pitch + yaw;

		motors->m1 = clamp_pwm(m1);
		motors->m2 = clamp_pwm(m2);
		motors->m3 = clamp_pwm(m3);
		motors->m4 = clamp_pwm(m4);

		motors->left_wheel = 1000;
		motors->right_wheel = 1000;
	}
	else if(mode == MODE_GROUND){
		motors->m1 = 1000;
		motors->m2 = 1000;
		motors->m3 = 1000;
		motors->m4 = 1000;

		float left = throttle + yaw;
		float right = throttle - yaw;

		motors->left_wheel = clamp_pwm(left);
		motors->right_wheel = clamp_pwm(right);
	}



}
