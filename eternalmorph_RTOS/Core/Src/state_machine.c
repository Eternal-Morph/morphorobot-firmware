/*
 * state_machine.c
 *
 *  Created on: Aug 18, 2026
 *      Author: vepex
 */


#include "state_machine.h"
#include "math.h"

void State_Init(FlightState_t* state, float angle_limit, uint32_t sensor_timeout_ms, uint32_t loop_timeout_ms){
	state->mode = STATE_DISARMED;
	state->fail_reason = FAIL_NONE;
	state->last_imu_tick = HAL_GetTick();

	state->angle_limit = angle_limit;
	state->sensor_timeout_ms = sensor_timeout_ms;
	state->loop_timeout_ms = loop_timeout_ms;
}

void State_Update(FlightState_t* state, float current_roll, float current_pitch, float dt){
	if(state->mode == STATE_ARMED){
		if((HAL_GetTick() - state->last_imu_tick) > state->sensor_timeout_ms){
			state->mode = STATE_FAILSAFE;
			state->fail_reason = FAIL_SENSOR_LOST;
		}
		if(fabs(current_roll) > state->angle_limit || fabs(current_pitch) > state->angle_limit){
			state->mode = STATE_FAILSAFE;
			state->fail_reason = FAIL_ANGLE_EXCEEDED;
		}
		if(dt > (state->loop_timeout_ms / 1000.0f)){
			state->mode = STATE_FAILSAFE;
			state->fail_reason = FAIL_LOOP_TIMEOUT;
		}
	}
	else{
		return;
	}

}

void State_Arm(FlightState_t* state){
	if(state->mode == STATE_DISARMED){
		state->mode = STATE_ARMED;
		state->armed_tick = HAL_GetTick();
	}
	else{
		return;
	}
}

void State_Disarm(FlightState_t* state){
	state->mode = STATE_DISARMED;
	state->fail_reason = FAIL_NONE;
}
