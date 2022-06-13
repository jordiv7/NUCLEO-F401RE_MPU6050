/*
 * MPU6050 Accelerometer/Gyroscope I2C Driver
 *
 *  Created on: Jun 7, 2022
 *      Author: Jordi
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stm32f4xx_hal.h>

// I2C ADDRESS
// https://www.addicore.com/v/vspfiles/downloadables/Product%20Downloadables/GY_521_Gyro_Accel/MPU_6000A.pdf

#define MPU6050_ADDR					(0b1101000 << 1)

// Registers
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#define MPU6050_REG_SELFTEST_X			0x0D
#define MPU6050_REG_SELFTEST_Y			0x0E
#define MPU6050_REG_SELFTEST_Z			0x0F
#define MPU6050_REG_SELFTEST_A			0x10
#define MPU6050_REG_CONFIG				0x1A
#define MPU6050_REG_GYRO_CONFIG			0x1B
#define MPU6050_REG_ACCEL_CONFIG		0x1C
#define MPU6050_REG_INT_PIN_CONFIG		0x37
#define MPU6050_REG_INT_ENABLE			0x38
#define MPU6050_REG_INT_STATUS			0x3A
#define MPU6050_REG_ACCEL_XOUT_H		0x3B
#define MPU6050_REG_ACCEL_XOUT_L		0x3C
#define MPU6050_REG_ACCEL_YOUT_H		0x3D
#define MPU6050_REG_ACCEL_YOUT_L		0x3E
#define MPU6050_REG_ACCEL_ZOUT_H		0x3F
#define MPU6050_REG_ACCEL_ZOUT_L		0x40
#define MPU6050_REG_TEMP_OUT_H			0x41
#define MPU6050_REG_TEMP_OUT_L			0x42
#define MPU6050_REG_GYRO_XOUT_H			0x43
#define MPU6050_REG_GYRO_XOUT_L			0x44
#define MPU6050_REG_GYRO_YOUT_H			0x45
#define MPU6050_REG_GYRO_YOUT_L			0x46
#define MPU6050_REG_GYRO_ZOUT_H			0x47
#define MPU6050_REG_GYRO_ZOUT_L			0x48
#define MPU6050_REG_SIGNAL_PATH_RESET	0x68
#define MPU6050_REG_USER_CONTROL		0x6A
#define MPU6050_REG_PWR_MGMT_1			0x6B
#define MPU6050_REG_PWR_MGMT_2			0x6C
#define MPU6050_REG_WHO_AM_I			0x75



typedef struct{

	// I2C handle
	I2C_HandleTypeDef *i2cHandle;

	// Test values
	uint8_t XA_TEST;
	uint8_t XG_TEST;
	uint8_t YA_TEST;
	uint8_t YG_TEST;
	uint8_t ZA_TEST;
	uint8_t ZG_TEST;

	uint16_t ACCEL_SSR;
	float GYRO_SSR;

	float ACCEL_X;
	float ACCEL_Y;
	float ACCEL_Z;

	float GYRO_X;
	float GYRO_Y;
	float GYRO_Z;

	float GYRO_ERR_X;
	float GYRO_ERR_Y;
	float GYRO_ERR_Z;

	float angle_pitch;
	float angle_roll;
}MPU6050;

// Initializer
uint8_t MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle);

// Read register
HAL_StatusTypeDef MPU6050_readRegister(MPU6050 *dev, uint8_t reg, uint8_t *data);

// Write register
HAL_StatusTypeDef MPU6050_writeRegister(MPU6050 *dev, uint8_t reg, uint8_t *data);

// Read Accelerometer
HAL_StatusTypeDef MPU6050_readAcceleration(MPU6050 *dev);
// Read Gyroscope
HAL_StatusTypeDef MPU6050_readGyroscope(MPU6050 *dev);

#endif /* INC_MPU6050_H_ */
