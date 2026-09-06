/*
 * mpu6050.c
 *
 *  Created on: Aug 13, 2026
 *      Author: vepex
 */

#include "mpu6050.h"
#include <math.h>

__attribute__((aligned(32))) uint8_t mpu_rx_buffer[32];

/* Wakes up the MPU6050 from sleep mode and configure */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
	// Data to write (0x00 wakes up the sensor)
	uint8_t temp_data = 0x00;

	if(HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &temp_data, 1, 100) != HAL_OK){
		return 0;
	}
	// 2. DLPF (Dijital Alçak Geçiren Filtre) Ayarı
	// 0x03 değeri -> 42Hz Bandwidth, 1kHz donanımsal çıkış hızı
	temp_data = 0x03;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG_REG, 1, &temp_data, 1, 100);

	// 3. Sample Rate (Örnekleme Hızı) Ayarı
	// 0x04 değeri -> 1000 Hz
	temp_data = 0x00;
	HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &temp_data, 1, 100);


		return 1;
}

/* Reads all data, converts them, and calculates angles */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *data, float dt){

	uint8_t accel_raw[6];

	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, accel_raw, 6, 100);
	// 8 bit sola kaydir(accel_raw[0] high sonra or islemine sok ( accel_raw[1] low)

	int16_t accel_x = (int16_t)(accel_raw[0] << 8 | accel_raw[1]);
	int16_t accel_y = (int16_t)(accel_raw[2] << 8 | accel_raw[3]);
	int16_t accel_z = (int16_t)(accel_raw[4] << 8 | accel_raw[5]);

	data->accel_x_g = accel_x / 16384.0f;
	data->accel_y_g = accel_y / 16384.0f;
	data->accel_z_g = accel_z / 16384.0f;

	uint8_t gyro_raw[6];

	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, gyro_raw, 6, 100);

	int16_t gyro_x = (int16_t)(gyro_raw[0] << 8 | gyro_raw[1]);
	int16_t gyro_y = (int16_t)(gyro_raw[2] << 8 | gyro_raw[3]);
	int16_t gyro_z = (int16_t)(gyro_raw[4] << 8 | gyro_raw[5]);

	data->gyro_x_dps = gyro_x / 131.0f;
	data->gyro_y_dps = gyro_y / 131.0f;
	data->gyro_z_dps = gyro_z / 131.0f;

	float rad_to_deg = 180.0f / 3.14159265f;

	// Roll (Yuvarlanma - X ekseni etrafında dönüş)
	data->roll = atan2(data->accel_x_g, data->accel_z_g) * rad_to_deg;

	// Pitch (Yunuslama - Y ekseni etrafında dönüş)
	data->pitch = atan2(-data->accel_y_g, sqrt(data->accel_x_g * data->accel_x_g + data->accel_z_g * data->accel_z_g)) * rad_to_deg;

	/* 4. COMPLEMENTARY FILTER */

	data->comp_roll = 0.96f * (data->comp_roll + (data->gyro_x_dps * dt)) + 0.04f * data->roll;
	data->comp_pitch = 0.96f * (data->comp_pitch + (data->gyro_y_dps * dt)) + 0.04f * data->pitch;

}

void MPU6050_Read_All_DMA(I2C_HandleTypeDef *hi2c){
	HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, mpu_rx_buffer, 14);

}

void MPU6050_Process_DMA_Data(MPU6050_t *data){
	// 1. İvme Verilerini Çıkar (0'dan 5. indexe kadar)
	int16_t accel_x = (int16_t)(mpu_rx_buffer[0] << 8 | mpu_rx_buffer[1]);
	int16_t accel_y = (int16_t)(mpu_rx_buffer[2] << 8 | mpu_rx_buffer[3]);
	int16_t accel_z = (int16_t)(mpu_rx_buffer[4] << 8 | mpu_rx_buffer[5]);

	data->accel_x_g = accel_x / 16384.0f;
	data->accel_y_g = accel_y / 16384.0f;
	data->accel_z_g = accel_z / 16384.0f;

	    // Not: mpu_rx_buffer[6] ve [7] Sıcaklık (TEMP_OUT) verisidir, atlıyoruz.

	    // 2. Jiroskop Verilerini Çıkar (8'den 13. indexe kadar)
	int16_t gyro_x = (int16_t)(mpu_rx_buffer[8] << 8 | mpu_rx_buffer[9]);
	int16_t gyro_y = (int16_t)(mpu_rx_buffer[10] << 8 | mpu_rx_buffer[11]);
	int16_t gyro_z = (int16_t)(mpu_rx_buffer[12] << 8 | mpu_rx_buffer[13]);

	data->gyro_x_dps = (gyro_x / 131.0f) - data->gyro_x_offset;
	data->gyro_y_dps = (gyro_y / 131.0f) - data->gyro_y_offset;
	data->gyro_z_dps = (gyro_z / 131.0f) - data->gyro_z_offset;
}

void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *data){
	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;

	for(int i=0;i<1000;i++){
		MPU6050_Read_All(hi2c, data, 0.0f);

		sum_x += data->gyro_x_dps;
		sum_y += data->gyro_y_dps;
		sum_z += data->gyro_z_dps;

		HAL_Delay(2);
	}

	data->gyro_x_offset = sum_x / 1000.0f;
	data->gyro_y_offset = sum_y / 1000.0f;
	data->gyro_z_offset = sum_z / 1000.0f;

}
