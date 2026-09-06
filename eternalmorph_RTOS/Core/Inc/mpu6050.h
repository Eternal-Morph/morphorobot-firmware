/*
 * mpu6050.h
 *
 *  Created on: Aug 13, 2026
 *      Author: vepex
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#define MPU6050_ADDR        (0x68 << 1)

#define PWR_MGMT_1_REG      0x6B
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43

#define SMPLRT_DIV_REG   0x19  // Sample Rate Divider (Örnekleme Hızı Bölücü)
#define CONFIG_REG       0x1A  // Configuration (DLPF Ayarları)

typedef struct{
	float accel_x_g;
	float accel_y_g;
	float accel_z_g;

	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;

	float roll;
	float pitch;

	float comp_roll;
	float comp_pitch;

	float gyro_x_offset;
	float gyro_y_offset;
	float gyro_z_offset;
} MPU6050_t;

/* Initialization function */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

/* Read all data and calculate angles */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data, float dt);

void MPU6050_Read_All_DMA(I2C_HandleTypeDef *hi2c);

void MPU6050_Process_DMA_Data(MPU6050_t *data);

void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *data);














#endif /* INC_MPU6050_H_ */
