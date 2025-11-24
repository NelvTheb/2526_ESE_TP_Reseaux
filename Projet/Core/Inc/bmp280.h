/*
 * bmp280.h
 *
 *  Created on: Nov 24, 2025
 *      Author: hugoc
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_


#include "stm32f4xx_hal.h"
#include"i2c.h"
#include <stdio.h>

#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CALIB_START  0x88
#define BMP280_CALIB_DATA_LEN   26

#define BMP280_I2C_ADDR  (0x77 << 1)

HAL_StatusTypeDef BMP280_Init(void);
HAL_StatusTypeDef BMP280_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
HAL_StatusTypeDef BMP280_ReadRegisters(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, uint16_t length);



#endif /* INC_BMP280_H_ */
