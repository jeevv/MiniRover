/*
 * mpu6050.c
 *
 *  Created on: Jul 11, 2024
 *      Author: bala
 */

#include "mpu6050.h"

HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *hi2c1)
{
	uint8_t check = 0;
	uint8_t data = 0;

	HAL_StatusTypeDef mpu_status = HAL_I2C_Mem_Read (hi2c1, MPU_ADDRESS,MPU_REG_AVAILABLE,1, &check, 1, 1000);

	if (check==0x68 && mpu_status == HAL_OK)
	{
		// Writing zeros to the register to wake up the sensor and
		// set clock frequency to 8 MHz

		HAL_StatusTypeDef power_status = HAL_I2C_Mem_Write(hi2c1, MPU_ADDRESS, MPU_REG_PWR_MGMT_1, 1,&data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		data = 0x07;
		HAL_StatusTypeDef sampling_status = HAL_I2C_Mem_Write(hi2c1, MPU_ADDRESS, MPU_REG_SMPL_RT_DIV	, 1, &data, 1, 1000);

		if(power_status == sampling_status)
		{
			return power_status;
		}

		else
		{
			return HAL_ERROR;
		}
	}

	return HAL_ERROR;

}

mpu_raw_data MPU_Read_Raw_Data(I2C_HandleTypeDef *hi2c1)
{
	mpu_raw_data raw_data;

	uint8_t rec_data[6];

	HAL_I2C_Mem_Read (hi2c1, MPU_ADDRESS, MPU_REG_ACC_X_H, 1, rec_data, 6, 1000);

	// Converting two separate 8-bit values into a single 16-bit value
	raw_data.ax = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	raw_data.ay = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	raw_data.az = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	// Doing the same for the gyroscope values
	HAL_I2C_Mem_Read (hi2c1, MPU_ADDRESS, MPU_REG_GYRO_X_H, 1, rec_data, 6, 1000);

	raw_data.gx = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	raw_data.gx  = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	raw_data.gx  = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	return raw_data;
}

mpu_scaled_data MPU_Scale_Data(mpu_raw_data raw_data)
{
	mpu_scaled_data scaled_data;

	// Dividing by 16384 to obtain actual value
	scaled_data.ax = (double) raw_data.ax/16384.0;
	scaled_data.ay = (double) raw_data.ay/16384.0;
	scaled_data.az = (double) raw_data.az/16384.0;

	// Dividing by 131.0 to obtain actual value
	scaled_data.gx = (double) raw_data.gx/131.0;
	scaled_data.gy = (double) raw_data.gy/131.0;
	scaled_data.gz = (double) raw_data.gz/131.0;

	return scaled_data;

}
