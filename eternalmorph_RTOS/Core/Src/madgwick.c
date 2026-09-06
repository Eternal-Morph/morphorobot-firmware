/*
 * madgwick.c
 *
 *  Created on: Aug 14, 2026
 *      Author: vepex
 */
#include "madgwick.h"
#include "math.h"

void Madgwick_Init(quaternion* Qua, float B){
	Qua->q0 = 1.0f;
	Qua->q1 = 0.0f;
	Qua->q2 = 0.0f;
	Qua->q3 = 0.0f;
	Qua->beta = B;
}


void Madgwick_Update(quaternion* Qua,float gyro_x,float gyro_y,float gyro_z,float accel_x,float accel_y,float accel_z,float dt){

	float norm;


	norm = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);

	if(norm == 0)
		return; //no sensor data

	accel_x /= norm;
	accel_y /= norm;
	accel_z /= norm;

	float q0 = Qua->q0;
	float q1 = Qua->q1;
	float q2 = Qua->q2;
	float q3 = Qua->q3;

	float _2q0 = 2.0f*q0;
	float _2q1 = 2.0f*q1;
	float _2q2 = 2.0f*q2;
	float _2q3 = 2.0f*q3;

	float f1 = _2q1 * q3 - _2q0 * q2 - accel_x;
	float f2 = _2q0 * q1 + _2q2 * q3 - accel_y;
	float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - accel_z;

	float s0 = -_2q2 * f1 + _2q1 * f2;
	float s1 =  _2q3 * f1 + _2q0 * f2 - 4.0f * q1 * f3;
	float s2 = -_2q0 * f1 + _2q3 * f2 - 4.0f * q2 * f3;
	float s3 =  _2q1 * f1 + _2q2 * f2;

	norm = sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
	s0 /= norm;
	s1 /= norm;
	s2 /= norm;
	s3 /= norm;

	float qDot0 = 0.5f * (-q1*gyro_x - q2*gyro_y - q3*gyro_z);
	float qDot1 = 0.5f * ( q0*gyro_x + q2*gyro_z - q3*gyro_y);
	float qDot2 = 0.5f * ( q0*gyro_y - q1*gyro_z + q3*gyro_x);
	float qDot3 = 0.5f * ( q0*gyro_z + q1*gyro_y - q2*gyro_x);

	qDot0 -= Qua->beta * s0;
	qDot1 -= Qua->beta * s1;
	qDot2 -= Qua->beta * s2;
	qDot3 -= Qua->beta * s3;

	Qua->q0 += qDot0 * dt;
	Qua->q1 += qDot1 * dt;
	Qua->q2 += qDot2 * dt;
	Qua->q3 += qDot3 * dt;

	norm = sqrtf(Qua->q0*Qua->q0 + Qua->q1*Qua->q1 + Qua->q2*Qua->q2 + Qua->q3*Qua->q3);

	Qua->q0 /= norm;
	Qua->q1 /= norm;
	Qua->q2 /= norm;
	Qua->q3 /= norm;

	/* Quaternion -> Euler angles (radyan -> derece) */
	float rad_to_deg = 180.0f / 3.14159265f;
	Qua->roll  = atan2f(2.0f * (Qua->q0 * Qua->q1 + Qua->q2 * Qua->q3), 1.0f - 2.0f * (Qua->q1 * Qua->q1 + Qua->q2 * Qua->q2)) * rad_to_deg;
	float sinp = 2.0f * (Qua->q0 * Qua->q2 - Qua->q3 * Qua->q1);
	if (sinp > 1.0f)  sinp = 1.0f;
	if (sinp < -1.0f) sinp = -1.0f;
	Qua->pitch = asinf(sinp) * rad_to_deg;
	Qua->yaw   = atan2f(2.0f * (Qua->q0 * Qua->q3 + Qua->q1 * Qua->q2), 1.0f - 2.0f * (Qua->q2 * Qua->q2 + Qua->q3 * Qua->q3)) * rad_to_deg;

}
