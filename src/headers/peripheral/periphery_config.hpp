/**
 * Defines structures and functions for configuring I2C buses, SPI devices, and
 * UART devices.
 */

#ifndef PERIPHERY_CONFIG_INCLUDED
#define PERIPHERY_CONFIG_INCLUDED

#ifdef __cplusplus
extern "C"
{
#endif

#include "periphery/i2c.h"
#include "periphery/spi.h"
#include "periphery/serial.h"

enum Endianness { LSB_FIRST, MSB_FIRST };

enum Parity { NONE, ODD, EVEN };

enum IdleLevel { LOW, HIGH };

enum CaptureEdge { RISING, FALLING };

enum BaudRate
{
  _50 = 50,
  _75 = 75,
  _110 = 110,
  _134 = 134,
  _150 = 150,
  _200 = 200,
  _300 = 300,
  _600 = 600,
  _1200 = 1200,
  _1800 = 1800,
  _2400 = 2400,
  _4800 = 4800,
  _9600 = 9600,
  _19200 = 19200,
  _38400 = 38400,
  _57600 = 57600,
  _115200 = 115200,
  _230400 = 230400,
  _460800 = 460800,
  _500000 = 500000,
  _576000 = 576000,
  _921600 = 921600,
  _1000000 = 1000000,
  _1152000 = 1152000,
  _1500000 = 1500000,
  _2000000 = 2000000
};

/**
 * SPI device settings.
 */
struct SpiConf
{
  IdleLevel idle_level = IdleLevel::HIGH;
  CaptureEdge capture_edge = CaptureEdge::RISING;
  unsigned int max_speed = 1000000;
  Endianness endianness = Endianness::MSB_FIRST;
  unsigned int word_size = 8;
};

/**
 * UART device settings.
 */
struct UartConf
{
  BaudRate baudrate = BaudRate::_9600;
  Parity parity = Parity::NONE;
  bool xonxoff = false;
};

#ifdef __cplusplus
}
#endif

#endif
