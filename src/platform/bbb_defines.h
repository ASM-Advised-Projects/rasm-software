/**
 * bbb_defines.h
 * 
 * Contains constants that describe the beagle bone black computer.
 */

#ifndef BBB_DEFINES_INCLUDED
#define BBB_DEFINES_INCLUDED

#define I2C_PREFIX "/dev/i2c-"
#define I2C_BUS1 0
#define I2C_BUS2 1
#define I2C_BUS3 0
//#define I2C_MAXDEVS 3

#define SPI_PREFIX "/dev/spidev"
#define SPI_BUS1_NUM 1
#define SPI_BUS2_NUM 2
#define SPI_BUS1DEV1_NUM 0
#define SPI_BUS1DEV2_NUM 1
#define SPI_BUS2DEV1_NUM 0
#define SPI_BUS2DEV2_NUM 1
//#define SPI_MAXDEVS 4

#define UART_PREFIX "/dev/ttyO"
#define UART_DEV1 0
#define UART_DEV2 1
#define UART_DEV3 2
#define UART_DEV4 3
#define UART_DEV5 4
#define UART_DEV6 5

#endif
