/*
 * ina226.h
 *
 *  Created on: Jan 7, 2024
 *      Author: DANILA
 */
#include "stm32f4xx_hal.h"

/*
 * init example
 * //START INA init
 * INA226_InitTypeDef INA226_Init_struct = {0};
 * //input
 * INA226_Init_struct.hi2c 			= hi2c1;
 * INA226_Init_struct.ID 			= 0x90;
 * INA226_Init_struct.Configuration = 0x43FF;
 * INA226_Init_struct.Calibration 	= 0x1400;
 * INA226_Init_struct.Mask 			= 0x8000;
 * INA226_Init_struct.Alert			= 0x2EE0;
 * INA226_Init(&INA226_Init_struct);
 * //END INA init
 */
typedef struct ina226_init_structure
{
	I2C_HandleTypeDef hi2c;
		// i2c header name
	uint16_t ID;
		// slave address page 18 on datasheet
	uint16_t Configuration;
		//
	uint16_t Calibration;
		//
	uint16_t Mask;
		//
	uint16_t Alert;
		//
} INA226_InitTypeDef;

void INA_Write_Register(I2C_HandleTypeDef hi2c, uint16_t DevAddress, uint8_t RegAddress, uint16_t Data);
void INA226_Init(INA226_InitTypeDef *INA226_Init);

uint16_t INA226_GetVoltage_hex(I2C_HandleTypeDef hi2c, uint16_t DevAddress);
uint16_t INA226_GetCurrent_hex(I2C_HandleTypeDef hi2c, uint16_t DevAddress);
float INA226_GetVoltage_float(I2C_HandleTypeDef hi2c, uint16_t DevAddress);
float INA226_GetCurrent_float(I2C_HandleTypeDef hi2c, uint16_t DevAddress);
uint16_t INA226_GetVoltageShunt(I2C_HandleTypeDef hi2c, uint16_t DevAddress);
