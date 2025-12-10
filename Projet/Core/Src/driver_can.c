#include "driver_can.h"

/* Handle CAN généré par CubeMX */
extern CAN_HandleTypeDef hcan1;
extern int32_t temp100;

/* Header CAN */
static CAN_TxHeaderTypeDef pHeader;

/* Boîte aux lettres */
static uint32_t pTxMailbox ;

/* Données CAN */
static uint8_t TxData[2];

void DRIVER_CAN_Init(void)
{
    /* Activation du module CAN */
    HAL_CAN_Start(&hcan1);

    /* Configuration de la trame CAN */
    pHeader .StdId = 0x61;                 // Mode automatique : Angle
    pHeader .ExtId = 0;
    pHeader .IDE   = CAN_ID_STD;            // ID standard
    pHeader .RTR   = CAN_RTR_DATA;          // Trame DATA
    pHeader .DLC   = 2;                     // D0 + D1
    pHeader .TransmitGlobalTime = DISABLE;


	DRIVER_CAN_SendAngle(0, POSITIVE);
	HAL_Delay(1000);
}

void DRIVER_CAN_SendAngle(uint8_t angle, uint8_t sign)
{
    /* Sécurité : angle max = 180° */
    if (angle > 180)
        angle = 180;

    TxData[0] = angle;   // D0 : angle
    TxData[1] = sign;    // D1 : signe (0 = +, 1 = -)

    HAL_CAN_AddTxMessage(&hcan1, &pHeader , TxData, &pTxMailbox );
}
