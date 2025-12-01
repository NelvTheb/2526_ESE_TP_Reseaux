#include "bmp280.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

// ---- Variables globales ----
int32_t t_fine;

uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;

uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

extern UART_HandleTypeDef huart4;
extern int cmd_index;
extern uint8_t uart4_rx;
extern char command[16];

// ---- Fonctions I2C ----
HAL_StatusTypeDef BMP280_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef BMP280_ReadRegisters(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, uint16_t length) {
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    ret = HAL_I2C_Master_Receive(hi2c, BMP280_I2C_ADDR, buffer, length, HAL_MAX_DELAY);
    return ret;
}

// ---- Initialisation ----
HAL_StatusTypeDef BMP280_Init(void) {
    uint8_t id;
    HAL_StatusTypeDef ret;

    // 1) Lecture de l'ID
    ret = BMP280_ReadRegisters(&hi2c1, BMP280_REG_ID, &id, 1);
    if (ret != HAL_OK) {
        printf("Erreur lecture ID BMP280\r\n");
        return ret;
    }
    printf("BMP280 ID = 0x%02X\r\n", id);

    if (id != 0x58) {
        printf("ID inattendu, ce n'est peut-être pas un BMP280\r\n");
        return HAL_ERROR;
    }

    // 2) Configuration CTRL_MEAS : Temp x2, Press x16, mode normal -> BMP280_Init_Config
    ret = BMP280_WriteRegister(&hi2c1, BMP280_REG_CTRL_MEAS, BMP280_Init_Config);
    if (ret != HAL_OK) {
        printf("Erreur écriture configuration CTRL_MEAS\r\n");
        return ret;
    }

    // 3) Vérification écriture
    uint8_t check;
    ret = BMP280_ReadRegisters(&hi2c1, BMP280_REG_CTRL_MEAS, &check, 1);
    if (ret != HAL_OK) {
        printf("Erreur lecture vérification configuration\r\n");
        return ret;
    }

    if (check == BMP280_Init_Config) {
        printf("Configuration appliquée: CTRL_MEAS = 0x%02X\r\n", check);
    } else {
        printf("Configuration incorrecte: lu = 0x%02X\r\n", check);
    }

    return HAL_OK;
}

// ---- Lecture des coefficients de calibration ----
HAL_StatusTypeDef BMP280_ReadCalibration(void)
{
    uint8_t buf[24];
    HAL_StatusTypeDef ret;

    ret = BMP280_ReadRegisters(&hi2c1, BMP280_REG_CALIB_START, buf, 24);
    if (ret != HAL_OK) return ret;

    dig_T1 = (buf[1] << 8) | buf[0];
    dig_T2 = (buf[3] << 8) | buf[2];
    dig_T3 = (buf[5] << 8) | buf[4];

    dig_P1 = (buf[7] << 8) | buf[6];
    dig_P2 = (buf[9] << 8) | buf[8];
    dig_P3 = (buf[11] << 8) | buf[10];
    dig_P4 = (buf[13] << 8) | buf[12];
    dig_P5 = (buf[15] << 8) | buf[14];
    dig_P6 = (buf[17] << 8) | buf[16];
    dig_P7 = (buf[19] << 8) | buf[18];
    dig_P8 = (buf[21] << 8) | buf[20];
    dig_P9 = (buf[23] << 8) | buf[22];

    return HAL_OK;
}

// ---- Lecture RAW ----
HAL_StatusTypeDef BMP280_ReadRaw(int32_t *raw_temp, int32_t *raw_press) {
    uint8_t buffer[6];
    HAL_StatusTypeDef ret;

    ret = BMP280_ReadRegisters(&hi2c1, BMP280_REG_PRESS_MSB, buffer, 6);
    if (ret != HAL_OK) return ret;

    *raw_press = (int32_t)(buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);
    *raw_temp  = (int32_t)(buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4);

    return HAL_OK;
}

// ---- Compensation température ----
int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
             ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;   // 0.01 °C
    return T;
}

// ---- Compensation pression ----
uint32_t bmp280_compensate_P_int32(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;

    var1 = ((t_fine >> 1) - (int32_t)64000);

    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);

    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
           ((((int32_t)dig_P2) * var1) >> 1)) >> 18;

    var1 = (((32768 + var1) * ((int32_t)dig_P1)) >> 15);
    if (var1 == 0) return 0;

    p = ((((uint32_t)1048576 - adc_P) - (var2 >> 12)));
    p = (p * 3125);

    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / ((uint32_t)var1)) * 2;

    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;

    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));

    return p; // Pa
}

// ---- Lecture + compensation unique ----
HAL_StatusTypeDef BMP280_ReadTempPressInt(int32_t* temperature_100, uint32_t* pressure_100)
{
    int32_t raw_T, raw_P;
    HAL_StatusTypeDef ret;

    ret = BMP280_ReadRaw(&raw_T, &raw_P);
    if (ret != HAL_OK) return ret;

    *temperature_100 = bmp280_compensate_T_int32(raw_T);
    *pressure_100    = bmp280_compensate_P_int32(raw_P);

    return HAL_OK;
}

// Commande rPi

void process_command(char *cmd)
{
    int32_t temp100;
    uint32_t press100;

    if (strcmp(cmd, "GET_T") == 0)
    {
        if (BMP280_ReadTempPressInt(&temp100, &press100) == HAL_OK)
        {
            char msg[16];
            // Format demandé : T=+12.50_C sur 10 caractères
            snprintf(msg, sizeof(msg), "T=%+02ld.%02ld_C",
                     temp100 / 100, temp100 % 100);

            HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    else if (strcmp(cmd, "GET_P") == 0)
    {
        if (BMP280_ReadTempPressInt(&temp100, &press100) == HAL_OK)
        {
            char msg[16];
            // Format : P=102300Pa (Pa = pression en Pa)
            snprintf(msg, sizeof(msg), "P=%06luPa",
                     press100);

            HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    else
    {
        char *err = "CMD_ERR";
        HAL_UART_Transmit(&huart4, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        // Echo immédiat pour voir ce qu'on tape dans minicom
        HAL_UART_Transmit(&huart4, &uart4_rx, 1, HAL_MAX_DELAY);

        if (uart4_rx != '\n' && uart4_rx != '\r')
        {
            if (cmd_index < sizeof(command) - 1)
            {
                command[cmd_index++] = uart4_rx;
            }
        }
        else
        {
            command[cmd_index] = '\0';

            // On traite la commande
            process_command(command);

            // IMPORTANT : vider le buffer !  Sinon on ne peut pas faire plusieurs requêtes
            memset(command, 0, sizeof(command));

            cmd_index = 0;
        }

        HAL_UART_Receive_IT(&huart4, &uart4_rx, 1);
    }
}
