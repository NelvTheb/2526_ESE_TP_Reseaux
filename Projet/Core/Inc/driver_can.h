/*
 * driver_can.h
 *
 *  Created on: Dec 10, 2025
 *      Author: hugoc
 */

#ifndef INC_DRIVER_CAN_H_
#define INC_DRIVER_CAN_H_

#include "can.h"
#include "stm32f4xx_hal.h"


//ON INVERSE LES DEUX ADRESSES POUR SUIVRE L'AFFICHAGE DU MOTEUR
#define NEGATIVE 0x00
#define POSITIVE 0x01

#define TEMP_MOT_COEFF 0.3

void DRIVER_CAN_Init(void);
void DRIVER_CAN_SendAngle(uint8_t angle, uint8_t sign);
void temp_to_angle_hot_or_cold(int32_t temp_100);

#endif /* INC_DRIVER_CAN_H_ */
