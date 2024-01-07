/*
 * ina226.c
 *
 *  Created on: Jan 7, 2024
 *      Author: DANILA
 */

#include "ina226.h"

void INA_Write_Register(I2C_HandleTypeDef hi2c, uint16_t DevAddress, uint8_t RegAddress, uint16_t Data)
{
	uint8_t data_i2c[3];
	data_i2c[0] = RegAddress;
	data_i2c[1] = (uint8_t)(Data>>8);
	data_i2c[2] = (uint8_t)Data;
	HAL_I2C_Master_Transmit	(&hi2c, DevAddress, data_i2c, 3, 100);
	HAL_I2C_Master_Receive	(&hi2c, DevAddress, data_i2c, 2, 100);
}

void INA226_Init(INA226_InitTypeDef *INA226_Init)
{
	INA_Write_Register(INA226_Init->hi2c, INA226_Init->ID, 0x00, INA226_Init->Configuration);
	INA_Write_Register(INA226_Init->hi2c, INA226_Init->ID, 0x05, INA226_Init->Calibration);
	INA_Write_Register(INA226_Init->hi2c, INA226_Init->ID, 0x06, INA226_Init->Mask);
	INA_Write_Register(INA226_Init->hi2c, INA226_Init->ID, 0x07, INA226_Init->Alert);
}

uint16_t INA226_GetVoltage_hex(I2C_HandleTypeDef hi2c, uint16_t DevAddress)
{
	uint8_t data_i2c[2];
	uint16_t voltage = 0;

	data_i2c[0] = 0x02;
	HAL_I2C_Master_Transmit(&hi2c, DevAddress, data_i2c, 1, 100);
	HAL_I2C_Master_Receive( &hi2c, DevAddress, data_i2c, 2, 100);
	voltage = (uint16_t)data_i2c[0]<<8;
	voltage |= (uint16_t)data_i2c[1];

	return voltage;
}

uint16_t INA226_GetCurrent_hex(I2C_HandleTypeDef hi2c, uint16_t DevAddress)
{
	uint8_t data_i2c[3];
	uint16_t current = 0;

	data_i2c[0] = 0x04;
	HAL_I2C_Master_Transmit(&hi2c, DevAddress, data_i2c, 1, 100);
	HAL_I2C_Master_Receive( &hi2c, DevAddress, data_i2c, 2, 100);
	current = (uint16_t)data_i2c[0]<<8;
	current |= (uint16_t)data_i2c[1];

	return current;
}

float INA226_GetVoltage_float(I2C_HandleTypeDef hi2c, uint16_t DevAddress)
{
	return (float)(int)INA226_GetVoltage_hex(hi2c, DevAddress) * 0.00125f;
}

float INA226_GetCurrent_float(I2C_HandleTypeDef hi2c, uint16_t DevAddress)
{
	return (float)(int)INA226_GetCurrent_hex(hi2c, DevAddress) * 0.004f;
}

uint16_t INA226_GetVoltageShunt(I2C_HandleTypeDef hi2c, uint16_t DevAddress)
{
	uint8_t data_i2c[2];
	uint16_t voltage = 0;

	data_i2c[0] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c, DevAddress, data_i2c, 1, 100);
	HAL_I2C_Master_Receive( &hi2c, DevAddress, data_i2c, 2, 100);
	voltage = (uint16_t)data_i2c[0]<<8;
	voltage |= (uint16_t)data_i2c[1];

	return voltage;
}
