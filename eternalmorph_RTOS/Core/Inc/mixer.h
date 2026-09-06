/*
 * mixer.h
 *
 *  Created on: Aug 17, 2026
 *      Author: vepex
 */

#ifndef INC_MIXER_H_
#define INC_MIXER_H_

#include "main.h"

typedef enum{
	MODE_AIR = 0,
	MODE_GROUND = 1
}RobotModel_t;

typedef struct{
	uint16_t m1;							// Air mode Front right BLDC
	uint16_t m2;							// Air mode Back right BLDC
	uint16_t m3;							// Air mode Back left BLDC
	uint16_t m4;							// Air mode Front right BLDC

	uint16_t left_wheel;					// Ground mode Left wheels
	uint16_t right_wheel;					// Ground mode Right wheels

}Motor_Output_t;


void Mixer_Init(Motor_Output_t* motors);

void Mixer_Update(Motor_Output_t* motors, RobotModel_t mode, float throttle, float roll, float pitch, float yaw, uint8_t is_armed);


#endif /* INC_MIXER_H_ */
