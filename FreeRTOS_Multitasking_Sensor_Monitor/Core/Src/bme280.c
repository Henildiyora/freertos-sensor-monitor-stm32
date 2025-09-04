#include "bme280.h"
#include <string.h>

// Structure to hold calibration data
typedef struct {
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
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_CalibData_t;

static BME280_CalibData_t calib_data;
static int32_t t_fine;

// Helper function to read calibration data
static HAL_StatusTypeDef BME280_ReadCalibrationData(I2C_HandleTypeDef *hi2c) {
    uint8_t calib[24];
    if (HAL_I2C_Mem_Read(hi2c, BME280_ADDRESS, BME280_REG_CALIB00, 1, calib, 24, 100) != HAL_OK) return HAL_ERROR;

    calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    calib_data.dig_P9 = (calib[23] << 8) | calib[22];

    uint8_t hum_calib[1];
    if (HAL_I2C_Mem_Read(hi2c, BME280_ADDRESS, 0xA1, 1, hum_calib, 1, 100) != HAL_OK) return HAL_ERROR;
    calib_data.dig_H1 = hum_calib[0];

    if (HAL_I2C_Mem_Read(hi2c, BME280_ADDRESS, BME280_REG_CALIB26, 1, calib, 7, 100) != HAL_OK) return HAL_ERROR;
    calib_data.dig_H2 = (calib[1] << 8) | calib[0];
    calib_data.dig_H3 = calib[2];
    calib_data.dig_H4 = (calib[3] << 4) | (calib[4] & 0x0F);
    calib_data.dig_H5 = (calib[5] << 4) | (calib[4] >> 4);
    calib_data.dig_H6 = calib[6];

    return HAL_OK;
}

HAL_StatusTypeDef BME280_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check_id;
    if (HAL_I2C_Mem_Read(hi2c, BME280_ADDRESS, BME280_REG_ID, 1, &check_id, 1, 100) != HAL_OK) return HAL_ERROR;
    if (check_id != 0x60) return HAL_ERROR; // BME280 ID is 0x60

    if (BME280_ReadCalibrationData(hi2c) != HAL_OK) return HAL_ERROR;

    uint8_t config_data;

    // Humidity oversampling x1
    config_data = 0x01;
    if (HAL_I2C_Mem_Write(hi2c, BME280_ADDRESS, BME280_REG_CTRL_HUM, 1, &config_data, 1, 100) != HAL_OK) return HAL_ERROR;

    // Temp/Pressure oversampling x1, normal mode
    config_data = 0x27; // 0b00100111
    if (HAL_I2C_Mem_Write(hi2c, BME280_ADDRESS, BME280_REG_CTRL_MEAS, 1, &config_data, 1, 100) != HAL_OK) return HAL_ERROR;

    // Standby 1000ms, filter off
    config_data = 0xA0; // 0b10100000
    if (HAL_I2C_Mem_Write(hi2c, BME280_ADDRESS, BME280_REG_CONFIG, 1, &config_data, 1, 100) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

// Compensation formulas from the BME280 datasheet
static float BME280_CompensateTemperature(int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

static float BME280_CompensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
    if (var1 == 0) return 0; // avoid exception caused by division by zero
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (float)p / 256.0f;
}

static float BME280_CompensateHumidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) *
                 (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                 ((int32_t)2097152)) * ((int32_t)calib_data.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (float)(v_x1_u32r >> 12) / 1024.0f;
}

HAL_StatusTypeDef BME280_ReadData(I2C_HandleTypeDef *hi2c, BME280_Data_t *data) {
    uint8_t raw_data[8];
    if (HAL_I2C_Mem_Read(hi2c, BME280_ADDRESS, BME280_REG_PRESS_MSB, 1, raw_data, 8, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    int32_t adc_P = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
    int32_t adc_T = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
    int32_t adc_H = (raw_data[6] << 8) | raw_data[7];

    data->temperature = BME280_CompensateTemperature(adc_T);
    data->pressure = BME280_CompensatePressure(adc_P) / 100.0f; // hPa
    data->humidity = BME280_CompensateHumidity(adc_H);

    return HAL_OK;
}
