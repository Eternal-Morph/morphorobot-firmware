/*
 * madgwick.h
 *
 *  Created on: Aug 14, 2026
 *      Author: vepex
 */

#ifndef INC_MADGWICK_H_
#define INC_MADGWICK_H_

#include "main.h"

typedef struct{

	float q0,q1,q2,q3;
	float beta;
	float roll,pitch,yaw;

}quaternion;


//Init → struct pointer alacak, beta değeri alacak, geri dönüş yok
void Madgwick_Init(quaternion* Qua, float B);

//Update → struct pointer, gyro x/y/z (rad/s), accel x/y/z, dt alacak
void Madgwick_Update(quaternion* Qua,float gyro_x,float gyro_y,float gyro_z,float accel_x,float accel_y,float accel_z,float dt);

#endif /* INC_MADGWICK_H_ */
