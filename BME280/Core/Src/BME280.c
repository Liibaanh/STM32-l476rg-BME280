/*
 * BME280.c
 *
 * Created on: Sep 27, 2026
 * Author: liibaanosman
 */

#include "BME280.h"
#include "string.h"
#include "stdio.h"

// Define precision mode selection for pressure compensation
//#define SUPPORT_64BIT 1
#define SUPPORT_32BIT 1

// External SPI handle defined in the main application (STM32CubeMX generated)
extern SPI_HandleTypeDef hspi1;

// Global sensor data and identification variables
extern float Temperature, Pressure, Humidity;
uint8_t chipID;

int32_t tRaw, pRaw, hRaw;
int32_t t_fine;

// Calibration parameters read from the sensor NVM
uint16_t dig_T1, dig_P1, dig_H1, dig_H3;
int16_t  dig_T2, dig_T3,
         dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9,
         dig_H2, dig_H4, dig_H5, dig_H6;

/**
 * @brief Write a single byte to a register via SPI.
 * @param reg Register address (MSB cleared for write).
 * @param data Byte value to write.
 */
void BME280_SPI_Write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg & 0x7F; // Clear bit 7 for SPI write
    buf[1] = data;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Set Chip Select (CS) low
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // Set Chip Select (CS) high
}

/**
 * @brief Read a single byte from a register via SPI.
 * @param reg Register address.
 * @return The byte read from the register.
 */
uint8_t BME280_SPI_Read(uint8_t reg)
{
    uint8_t txBuf[2] = { reg | 0x80, 0x00 }; // Set bit 7 for SPI read
    uint8_t rxBuf[2] = { 0, 0 };

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);   // Set CS low
    HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);     // Set CS high

    return rxBuf[1];   // Byte 0 is dummy during address transmission, Byte 1 contains data
}

/**
 * @brief Read multiple consecutive bytes from a starting register via SPI.
 * @param reg Starting register address.
 * @param data Pointer to buffer to store received data.
 * @param len Number of bytes to read.
 */
void BME280_SPI_ReadMulti(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t txBuf = reg | 0x80; // Set bit 7 for SPI read

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Set CS low
    HAL_SPI_Transmit(&hspi1, &txBuf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // Set CS high
}

/**
 * @brief Read factory calibration trimming parameters from sensor NVM.
 */
void TrimRead(void)
{
    uint8_t trimdata[33];

    BME280_SPI_ReadMulti(0x88, trimdata, 26);        // Read temperature and pressure coefficients (0x88..0xA1)
    BME280_SPI_ReadMulti(0xE1, trimdata + 26, 7);    // Read humidity coefficients (0xE1..0xE7)

    dig_T1 = (trimdata[1]<<8) | trimdata[0];
    dig_T2 = (trimdata[3]<<8) | trimdata[2];
    dig_T3 = (trimdata[5]<<8) | trimdata[4];
    dig_P1 = (trimdata[7]<<8) | trimdata[6];
    dig_P2 = (trimdata[9]<<8) | trimdata[8];
    dig_P3 = (trimdata[11]<<8) | trimdata[10];
    dig_P4 = (trimdata[13]<<8) | trimdata[12];
    dig_P5 = (trimdata[15]<<8) | trimdata[14];
    dig_P6 = (trimdata[17]<<8) | trimdata[16];
    dig_P7 = (trimdata[19]<<8) | trimdata[18];
    dig_P8 = (trimdata[21]<<8) | trimdata[20];
    dig_P9 = (trimdata[23]<<8) | trimdata[22];
    dig_H1 = trimdata[25];                                                    // 0xA1
    dig_H2 = (int16_t)((trimdata[27]<<8) | trimdata[26]);                     // 0xE1/0xE2
    dig_H3 = trimdata[28];                                                    // 0xE3
    dig_H4 = (int16_t)(((int8_t)trimdata[29] * 16) | (trimdata[30] & 0x0F));  // 0xE4/0xE5
    dig_H5 = (int16_t)(((int8_t)trimdata[31] * 16) | (trimdata[30] >> 4));    // 0xE6/0xE5
    dig_H6 = (int8_t)trimdata[32];                                            // 0xE7
}

/**
 * @brief Configure BME280 sensor parameters (oversampling, mode, standby time, filter).
 * @param osrs_t Temperature oversampling.
 * @param osrs_p Pressure oversampling.
 * @param osrs_h Humidity oversampling.
 * @param mode Power mode.
 * @param t_sb Standby time.
 * @param filter IIR filter coefficient.
 * @return 0 on success, -1 if verification fails.
 */
int BME280_config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
    uint8_t datatowrite = 0;
    uint8_t datacheck = 0;

    // 1. Reset sensor via soft-reset command
    BME280_SPI_Write(RESET_REG, 0xB6);
    HAL_Delay(100);

    // 2. Read calibration values after reset
    TrimRead();

    // 3. Configure humidity oversampling (ctrl_hum register, 0xF2)
    datatowrite = osrs_h;
    BME280_SPI_Write(CTRL_HUM_REG, datatowrite);
    HAL_Delay(100);

    datacheck = BME280_SPI_Read(CTRL_HUM_REG);
    if (datacheck != datatowrite) return -1;

    // 4. Configure standby and filter (config register, 0xF5)
    datatowrite = (t_sb << 5) | (filter << 2);
    BME280_SPI_Write(CONFIG_REG, datatowrite);
    HAL_Delay(100);

    datacheck = BME280_SPI_Read(CONFIG_REG);
    if (datacheck != datatowrite) return -1;

    // 5. Configure measurement control and mode (ctrl_meas register, 0xF4) - written last as it starts measurement
    datatowrite = (osrs_t << 5) | (osrs_p << 2) | mode;
    BME280_SPI_Write(CTRL_MEAS_REG, datatowrite);
    HAL_Delay(100);

    datacheck = BME280_SPI_Read(CTRL_MEAS_REG);
    if (datacheck != datatowrite) return -1;

    return 0;
}

/************* COMPENSATION CALCULATION AS PER DATASHEET (page 25) **************************/

/**
 * @brief Compensate raw temperature data using integer math.
 * @param adc_T Raw ADC temperature value.
 * @return Temperature in DegC (resolution 0.01 DegC, e.g., 5123 = 51.23 DegC).
 */
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

#if SUPPORT_64BIT
/**
 * @brief Compensate raw pressure data using 64-bit integer math.
 * @param adc_P Raw ADC pressure value.
 * @return Pressure in Pa as unsigned 32-bit integer in Q24.8 format.
 */
uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0)
    {
        return 0; // Avoid division by zero exception
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;
}

#elif SUPPORT_32BIT
/**
 * @brief Compensate raw pressure data using 32-bit integer math.
 * @param adc_P Raw ADC pressure value.
 * @return Pressure in Pa (e.g., 96386 = 963.86 hPa).
 */
uint32_t BME280_compensate_P_int32(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0; // Avoid division by zero exception
    }
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}
#endif

/**
 * @brief Compensate raw humidity data using integer math.
 * @param adc_H Raw ADC humidity value.
 * @return Humidity in %RH as unsigned 32-bit integer in Q22.10 format (e.g., 47445 = 46.333 %RH).
 */
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}
/*********************************************************************************************************/

/**
 * @brief Read raw sensor measurements, apply compensation formulas, and update global variables.
 */
void BME280_Measure(void)
{
    uint8_t RawData[8];

    // 1. Verify chip ID (BME280 ID is 0x60)
    chipID = BME280_SPI_Read(ID_REG);

    if (chipID == 0x60)
    {
        // 2. Read 8 raw data bytes starting from PRESS_MSB_REG
        BME280_SPI_ReadMulti(PRESS_MSB_REG, RawData, 8);

        // 3. Extract 20-bit pressure and temperature, and 16-bit humidity raw values
        pRaw = (RawData[0] << 12) | (RawData[1] << 4) | (RawData[2] >> 4);
        tRaw = (RawData[3] << 12) | (RawData[4] << 4) | (RawData[5] >> 4);
        hRaw = (RawData[6] << 8) | (RawData[7]);

        // 4. Compensate temperature
        if (tRaw == 0x80000) Temperature = 0; // Measurement disabled
        else
        {
            Temperature = BME280_compensate_T_int32(tRaw) / 100.0;
        }

        // 5. Compensate pressure
        if (pRaw == 0x80000) Pressure = 0; // Measurement disabled
        else
        {
        #if SUPPORT_64BIT
            Pressure = BME280_compensate_P_int64(pRaw) / 256.0;
        #elif SUPPORT_32BIT
            Pressure = BME280_compensate_P_int32(pRaw);
        #endif
        }

        // 6. Compensate humidity
        if (hRaw == 0x8000) Humidity = 0; // Measurement disabled
        else
        {
            Humidity = bme280_compensate_H_int32(hRaw) / 1024.0;
        }
    }
    else
    {
        Temperature = Pressure = Humidity = 0;
    }
}
