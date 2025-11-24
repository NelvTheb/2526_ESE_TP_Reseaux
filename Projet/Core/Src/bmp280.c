#include "bmp280.h"




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

    // 2) Configuration CTRL_MEAS : Temp x2, Press x16, mode normal -> 0x57
    ret = BMP280_WriteRegister(&hi2c1, BMP280_REG_CTRL_MEAS, 0x57);
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

    if (check == 0x57) {
        printf("Configuration appliquée: CTRL_MEAS = 0x%02X\r\n", check);
    } else {
        printf("Configuration incorrecte: lu = 0x%02X\r\n", check);
    }

    return HAL_OK;
}






// Ecrit 1 octet 'value' dans le registre 'reg'
HAL_StatusTypeDef BMP280_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

// Lit 'length' octets à partir du registre 'reg' dans 'buffer'
HAL_StatusTypeDef BMP280_ReadRegisters(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buffer, uint16_t length) {
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(hi2c, BMP280_I2C_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    ret = HAL_I2C_Master_Receive(hi2c, BMP280_I2C_ADDR, buffer, length, HAL_MAX_DELAY);
    return ret;
}


