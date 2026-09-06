/*
 * state_machine.h
 *
 *  Created on: Aug 18, 2026
 *      Author: vepex
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "main.h"


typedef enum{
	STATE_DISARMED = 0,
	STATE_ARMED,
	STATE_FAILSAFE
}FlightMode_t;

typedef enum{
	FAIL_NONE = 0,								//safe
	FAIL_SENSOR_LOST,							//no IMU data
	FAIL_ANGLE_EXCEEDED,						//excessive slope
	FAIL_LOOP_TIMEOUT							//the control loop stucked
}FailReason_t;

typedef struct{
	FlightMode_t mode;
	FailReason_t fail_reason;

	uint32_t last_imu_tick;
	uint32_t armed_tick;

	float angle_limit;
	uint32_t sensor_timeout_ms;
	uint32_t loop_timeout_ms;
}FlightState_t;

void State_Init(FlightState_t* state, float angle_limit, uint32_t sensor_timeout_ms, uint32_t loop_timeout_ms);

void State_Update(FlightState_t* state, float current_roll, float current_pitch, float dt);

void State_Arm(FlightState_t* state);

void State_Disarm(FlightState_t* state);

#endif /* INC_STATE_MACHINE_H_ */
