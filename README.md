# STM32-l476rg-BME280

# STM32-L476RG-BME280

A driver for the Bosch BME280 temperature, humidity and pressure sensor,
written for the STM32 NUCLEO-L476RG board using STM32CubeIDE and the STM32 HAL library.

The project communicates with the sensor over SPI and outputs the measured values
(temperature in °C, relative humidity in %, pressure in hPa).

## Hardware
- STM32 NUCLEO-L476RG
- BME280 sensor module (SPI mode)

## Wiring (SPI)

| BME280 pin | Nucleo pin | Note                  |
|------------|------------|-----------------------|
| VCC        | 3V3        |                       |
| GND        | GND        |                       |
| SCK / SCL  | PA5        | SPI1_SCK              |
| SDI / SDA  | PA7        | SPI1_MOSI             |
| SDO        | PA6        | SPI1_MISO             |
| CSB        | PB6        | Chip select (GPIO)    |

## Build
Open the project in STM32CubeIDE, build it, and flash it to the board.
The CubeMX configuration is in `BME280.ioc`.

## License
MIT
