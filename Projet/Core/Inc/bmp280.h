/*
 * bmp280.h
 *
 *  Created on: Nov 24, 2025
 *      Author: hugoc
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <stdio.h>

#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CALIB_START  0x88
#define BMP280_CALIB_DATA_LEN   24

#define BMP280_I2C_ADDR  (0x77 << 1)

#define BMP280_Init_temp 0b010 // [7:5]
#define BMP280_Init_press 0b101 // [4:2]
#define BMP280_Init_mode 0b11 // [1:0]

#define BMP280_Init_Config \
    ((BMP280_Init_temp  << 5) | \
     (BMP280_Init_press << 2) | \
     (BMP280_Init_mode))

// Coefficients de calibration
extern uint16_t dig_T1;
extern int16_t  dig_T2;
extern int16_t  dig_T3;
extern uint16_t dig_P1;
extern int16_t  dig_P2;
extern int16_t  dig_P3;
extern int16_t  dig_P4;
extern int16_t  dig_P5;
extern int16_t  dig_P6;
extern int16_t  dig_P7;
extern int16_t  dig_P8;
extern int16_t  dig_P9;

// t_fine global pour la pression
extern int32_t t_fine;

// Fonctions
HAL_StatusTypeDef BMP280_Init(void);
HAL_StatusTypeDef BMP280_ReadCalibration(void);
HAL_StatusTypeDef BMP280_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
HAL_StatusTypeDef BMP280_ReadRegisters(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, uint16_t length);
HAL_StatusTypeDef BMP280_ReadRaw(int32_t *raw_temp, int32_t *raw_press);

// Compensation 32 bits
int32_t bmp280_compensate_T_int32(int32_t adc_T);
uint32_t bmp280_compensate_P_int32(int32_t adc_P);

// Lecture + compensation en entier
HAL_StatusTypeDef BMP280_ReadTempPressInt(int32_t* temperature_100, uint32_t* pressure_100);

#endif /* INC_BMP280_H_ */
