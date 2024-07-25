/*
 * mpu6050.h
 *
 *  Created on: Jul 11, 2024
 *      Author: bala
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifndef __STM32F4xx_H
#include "stm32f4xx_hal.h"
#endif

//I2C address of the MPU6050
#define MPU_ADDRESS 0x75
#define MPU_WHO_AM_I_VALUE 0x68
// Register to check if MPU is working
#define MPU_REG_AVAILABLE 0x75
// Register to turn on sensor and set clock rate
#define MPU_REG_PWR_MGMT_1 0x6b
// Register to reduce sampling rate
#define MPU_REG_SMPL_RT_DIV 0x19
// Accelerometer and gyroscope configuration register
#define MPU_REG_GYRO_CONFIG 0x1b
#define MPU_REG_ACC_CONFIG 0x1c
// Starting address of the six registers storing accelerometer values
#define MPU_REG_ACC_X_H 0x3b
// Starting address of the six registers storing gyroscope values
#define MPU_REG_GYRO_X_H 0x43

struct mpu_raw_data{
	int16_t ax, ay, az, gx, gy, gz;
};

struct mpu_scaled_data
{
	double ax, ay, az, gx, gy, gz;
};

typedef struct mpu_raw_data mpu_raw_data;
typedef struct mpu_scaled_data mpu_scaled_data;

HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *);
mpu_raw_data MPU_Read_Raw_Data(I2C_HandleTypeDef *);
mpu_scaled_data MPU_Scale_Data(mpu_raw_data);

#endif /* INC_MPU6050_H_ */
