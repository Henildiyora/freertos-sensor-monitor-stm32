#ifndef BME280_H_
#define BME280_H_

#include "stm32f4xx_hal.h"

// BME280 I2C address
// SDO pin is low (0x76 << 1)
#define BME280_ADDRESS 0xEC

// BME280 Registers
#define BME280_REG_ID         0xD0
#define BME280_REG_RESET      0xE0
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_STATUS     0xF3
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_PRESS_MSB  0xF7
#define BME280_REG_CALIB00    0x88
#define BME280_REG_CALIB26    0xE1

// Data structure to hold sensor readings
typedef struct {
    float temperature;
    float humidity;
    float pressure;
} BME280_Data_t;



HAL_StatusTypeDef BME280_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef BME280_ReadData(I2C_HandleTypeDef *hi2c, BME280_Data_t *data);

#endif
