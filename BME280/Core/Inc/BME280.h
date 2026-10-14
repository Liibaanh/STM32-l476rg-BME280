/*
 * BME280.h
 *
 * Created on: Sep 27, 2026
 * Author: liibaanosman
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "stm32l4xx_hal.h"

// Public Function Prototypes


/**
 * @brief Write a single byte to a register via SPI.
 * @param reg Register address.
 * @param data Value to write.
 */
void BME280_SPI_Write(uint8_t reg, uint8_t data);

/**
 * @brief Read a single byte from a register via SPI.
 * @param reg Register address.
 * @return Value read from the register.
 */
uint8_t BME280_SPI_Read(uint8_t reg);

/**
 * @brief Read multiple consecutive bytes via SPI.
 * @param reg Starting register address.
 * @param data Pointer to buffer for received data.
 * @param len Number of bytes to read.
 */
void BME280_SPI_ReadMulti(uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief Configure sensor oversampling, power mode, standby time, and filter.
 * @param osrs_t Temperature oversampling.
 * @param osrs_p Pressure oversampling.
 * @param osrs_h Humidity oversampling.
 * @param mode Power mode.
 * @param t_sb Standby time.
 * @param filter IIR filter coefficient.
 * @return 0 on success, -1 on failure.
 */
int BME280_config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter);

/**
 * @brief Read NVM calibration trimming parameters.
 */
void TrimRead(void);

/**
 * @brief Perform a measurement, read raw data, and calculate compensated values.
 */
void BME280_Measure(void);


// Configuration Settings

// Oversampling Definitions

#define OSRS_OFF        0x00
#define OSRS_1          0x01
#define OSRS_2          0x02
#define OSRS_4          0x03
#define OSRS_8          0x04
#define OSRS_16         0x05

// Power Mode Definitions

#define MODE_SLEEP      0x00
#define MODE_FORCED     0x01
#define MODE_NORMAL     0x03

// Standby Time Settings in Normal Mode

#define T_SB_0p5        0x00
#define T_SB_62p5       0x01
#define T_SB_125        0x02
#define T_SB_250        0x03
#define T_SB_500        0x04
#define T_SB_1000       0x05
#define T_SB_10         0x06
#define T_SB_20         0x07

// IIR Filter Coefficients

#define IIR_OFF         0x00
#define IIR_2           0x01
#define IIR_4           0x02
#define IIR_8           0x03
#define IIR_16          0x04

// BME280 Register Map

#define ID_REG          0xD0
#define RESET_REG       0xE0
#define CTRL_HUM_REG    0xF2
#define STATUS_REG      0xF3
#define CTRL_MEAS_REG   0xF4
#define CONFIG_REG      0xF5
#define PRESS_MSB_REG   0xF7

// Global Variable Declarations

extern float Temperature, Pressure, Humidity;
extern uint8_t chipID;

#endif /* INC_BME280_H_ */
