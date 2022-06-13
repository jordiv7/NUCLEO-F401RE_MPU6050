/*
 * MPU6050.c
 *
 *  Created on: Jun 7, 2022
 *      Author: Jordi
 */

#include "MPU6050.h"

uint8_t MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;

	dev->ACCEL_X = 0.0f;
	dev->ACCEL_Y = 0.0f;
	dev->ACCEL_Z = 0.0f;

	dev->GYRO_X = 0.0f;
	dev->GYRO_Y = 0.0f;
	dev->GYRO_Z = 0.0f;

	dev->GYRO_ERR_X = 0.0f;
	dev->GYRO_ERR_Y = 0.0f;
	dev->GYRO_ERR_Z = 0.0f;

	dev->angle_pitch = 0.0f;
	dev->angle_roll = 0.0f;

	uint8_t errNum = 0;
	uint8_t regData;
	HAL_StatusTypeDef status;

	// Check if MPU6050 is connected
	status = MPU6050_readRegister(dev, MPU6050_REG_WHO_AM_I, &regData);
	errNum += (status != HAL_OK);

	if (regData != MPU6050_ADDR >> 1)
		return 255;

	// Get Accelerometer full scale range
	status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_CONFIG, &regData);
	errNum += (status != HAL_OK);

	// Bit select Bit 4 and Bit 3
	regData = (regData & 0b00011000) >> 3;
	if (regData == 0)
		dev->ACCEL_SSR = 16384;
	else if (regData == 1)
		dev->ACCEL_SSR = 8192;
	else if (regData == 2)
		dev->ACCEL_SSR = 4096;
	else if (regData == 3)
		dev->ACCEL_SSR = 2048;

	// Get Gyroscope full scale range
	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_CONFIG, &regData);
	errNum += (status != HAL_OK);

	// Bit select Bit 4 and Bit 3
	regData = (regData & 0b00011000) >> 3;
	if (regData == 0)
		dev->GYRO_SSR = 131;
	else if (regData == 1)
		dev->GYRO_SSR = 65.5f;
	else if (regData == 2)
		dev->GYRO_SSR = 32.8f;
	else if (regData ==3)
		dev->GYRO_SSR = 16.4f;

	// Get Self Test Values
	uint8_t selfTest_tempXA, selfTest_tempYA, selfTest_tempZA;
	status = MPU6050_readRegister(dev, MPU6050_REG_SELFTEST_X, &regData);
	errNum += (status != HAL_OK);

	dev->XG_TEST = (regData & 0x1F);
	selfTest_tempXA = (regData & 0xE0) >> 3;

	status = MPU6050_readRegister(dev, MPU6050_REG_SELFTEST_Y, &regData);
	errNum += (status != HAL_OK);

	dev->YG_TEST = (regData & 0x1F);
	selfTest_tempYA = (regData & 0xE0) >> 3;

	status = MPU6050_readRegister(dev, MPU6050_REG_SELFTEST_Z, &regData);
	errNum += (status != HAL_OK);

	dev->ZG_TEST = (regData & 0x1F);
	selfTest_tempZA = (regData & 0xE0) >> 3;

	status = MPU6050_readRegister(dev, MPU6050_REG_SELFTEST_A, &regData);
	errNum += (status != HAL_OK);

	selfTest_tempXA |= (regData & 0x30) >> 4;
	selfTest_tempYA |= (regData & 0b00001100) >> 2;
	selfTest_tempZA |= (regData & 0x03);

	dev->XA_TEST = selfTest_tempXA & 0x1F;
	dev->YA_TEST = selfTest_tempYA & 0x1F;
	dev->ZA_TEST = selfTest_tempZA & 0x1F;

	// Get low pass filter settings
	status = MPU6050_readRegister(dev, MPU6050_REG_CONFIG, &regData);
	errNum += (status != HAL_OK);

	// Set low pass filter settings
	regData &= ~0xF8;
	regData |= 0x03;
	status = MPU6050_writeRegister(dev, MPU6050_REG_CONFIG, &regData);
	errNum += (status != HAL_OK);

	// Turn on device
	regData = 0x00;
	status = MPU6050_writeRegister(dev, MPU6050_REG_PWR_MGMT_1, &regData);
	errNum += (status != HAL_OK);

	// Zero the Gyro
	for (uint16_t calc_ind = 0; calc_ind < 1000; calc_ind++){
		MPU6050_readGyroscope(dev);
		dev->GYRO_ERR_X += dev->GYRO_X;
		dev->GYRO_ERR_Y += dev->GYRO_Y;
		dev->GYRO_ERR_Z += dev->GYRO_Z;
		HAL_Delay(3);
	}
	dev->GYRO_ERR_X /= 1000;
	dev->GYRO_ERR_Y /= 1000;
	dev->GYRO_ERR_Z /= 1000;

	return errNum;
}

HAL_StatusTypeDef MPU6050_readRegister(MPU6050 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_writeRegister(MPU6050 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write(dev->i2cHandle, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MPU6050_readAcceleration(MPU6050 *dev){
	uint8_t regData[2];
	int16_t tempACCEL;
	// Get upper and lower X acceleration values
	HAL_StatusTypeDef status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_XOUT_H, &regData[0]);
	status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_XOUT_L, &regData[1]);
	tempACCEL = ((int16_t)regData[0] << 8) | regData[1];
	dev->ACCEL_X = tempACCEL/dev->ACCEL_SSR;

	// Get upper and lower Y acceleration values
	status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_YOUT_H, &regData[0]);
	status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_YOUT_L, &regData[1]);
	tempACCEL = ((int16_t)regData[0] << 8) | regData[1];
	dev->ACCEL_Y = tempACCEL/dev->ACCEL_SSR;

	// Get upper and lower Z acceleration values
	status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_ZOUT_H, &regData[0]);
	status = MPU6050_readRegister(dev, MPU6050_REG_ACCEL_ZOUT_L, &regData[1]);
	tempACCEL = ((int16_t)regData[0] << 8) | regData[1];
	dev->ACCEL_Z = tempACCEL/dev->ACCEL_SSR;

	return status;
}

HAL_StatusTypeDef MPU6050_readGyroscope(MPU6050 *dev){
	uint8_t regData[2];
	int16_t tempGyro;
	HAL_StatusTypeDef status;

	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_XOUT_H, &regData[0]);
	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_XOUT_L, &regData[1]);
	tempGyro = ((int16_t)regData[0] << 8) | regData[1];
	dev->GYRO_X = tempGyro/dev->GYRO_SSR;

	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_YOUT_H, &regData[0]);
	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_YOUT_L, &regData[1]);
	tempGyro = ((int16_t)regData[0] << 8) | regData[1];
	dev->GYRO_Y = tempGyro/dev->GYRO_SSR;

	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_ZOUT_H, &regData[0]);
	status = MPU6050_readRegister(dev, MPU6050_REG_GYRO_ZOUT_L, &regData[1]);
	tempGyro = ((int16_t)regData[0] << 8) | regData[1];
	dev->GYRO_Z = tempGyro/dev->GYRO_SSR;

	return status;
}
