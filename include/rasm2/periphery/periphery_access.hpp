/**
 * Defines the PeripheryAccess (singleton) class.
 */

#ifndef PERIPHERY_ACCESS_H
#define PERIPHERY_ACCESS_H

#include "periphery/i2c.h"
#include "periphery/spi.h"
#include "periphery/serial.h"
#include "periphery_config.hpp"
#include "configuration.hpp"
#include "logging.hpp"

#include "Poco/Format.h"

#include <map>

using std::string;
using Poco::format;

/**
 * This class is a singleton that provides access to a set of I2C buses, SPI
 * devices, and UART devices. Access is provided with contiguous bus/device
 * indices which correspond to i2c_t, spi_t, and serial_t structures.
 *
 * Note that a bus/device instance is only created when it is requested.
 */
class PeripheryAccess
{
private:
  // bus index to i2c bus device file mapping
  std::map<int, string> i2c_files;

  // bus and device index to spi device file mapping
  std::map<std::pair<int, int>, string> spi_files;

  // device index to uart device file mapping
  std::map<int, string> uart_files;

  int i2c_buses;     // number of i2c buses
  int spi_buses;     // number of spi buses
  int *spi_devices;  // number of spi devices for each bus
  int uart_devices;  // number of uart devices

  /**
   * Constructs this access manager using the PERIPHERAL configuration group.
   */
  PeripheryAccess()
  {
    // get configuration group
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
      get_config_group(ConfigurationManager::Group::PERIPHERY);

    string configkey;

    // populate i2c device file map
    i2c_buses = configs->getInt("i2c.bus_count");
    for (int bus = 0; bus < i2c_buses; bus++)
    {
      configkey = format("i2c.bus%d_filepath", bus);
      i2c_files[bus] = configs->getString(configkey);
    }

    // populate spi device file map
    spi_buses = configs->getInt("spi.bus_count");
    spi_devices = new int[spi_buses];
    for (int bus = 0; bus < spi_buses; bus++)
    {
      configkey = format("spi.bus%d.device_count", bus);
      spi_devices[bus] = configs->getInt(configkey);
      for (int device = 0; device < spi_devices[bus]; device++)
      {
        configkey = format("spi.bus%d.device%d_filepath", bus, device);
        std::pair<int, int> busdev = std::pair<int, int>(bus, device);
        spi_files[busdev] = configs->getString(configkey);
      }
    }

    // populate uart device file map
    uart_devices = configs->getInt("uart.device_count");
    for (int device = 0; device < uart_devices; device++)
    {
      configkey = format("uart.device%d_filepath", device);
      uart_files[device] = configs->getString(configkey);
    }
  }

public:
  PeripheryAccess(const PeripheryAccess &) = delete;
  void operator=(const PeripheryAccess &) = delete;

  /**
   * Returns this singleton's instance.
   */
  static PeripheryAccess& get_instance()
  {
    static PeripheryAccess instance;
    return instance;
  }

  /**
   * Returns the proper spi mode integer based on the idle level and capture edge.
   */
  static int get_spi_mode(IdleLevel &idle, CaptureEdge &edge)
  {
    if (idle == IdleLevel::LOW && edge == CaptureEdge::RISING)
      return 0;

    if (idle == IdleLevel::LOW && edge == CaptureEdge::FALLING)
      return 1;

    if (idle == IdleLevel::HIGH && edge == CaptureEdge::FALLING)
      return 2;

    return 3;
  }

  /**
   * Returns the spi bit order as an spi_bit_order_t structure that represents
   * the given endianness.
   */
  static spi_bit_order_t get_spi_bit_order(Endianness order)
  {
    if (order == Endianness::LSB_FIRST)
      return LSB_FIRST;

    return MSB_FIRST;
  }

  /**
   * Returns the uart parity mode as a serial_parity_t structure that represents
   * the given parity.
   */
  static serial_parity_t get_serial_parity(Parity parity)
  {
    switch (parity)
    {
      case NONE:
        return PARITY_NONE;
      case ODD:
        return PARITY_ODD;
      case EVEN:
        return PARITY_EVEN;
    }
  }

  /**
   * Initializes the I2C handle structure pointed to by i2c_bus for the given bus
   * index if the corresponding i2c bus exists. Bus indices are incremental and
   * start at 0.
   *
   * Returns true if i2c_bus was successfully initialized; false otherwise.
   */
  bool init_i2c_bus(int bus_index, i2c_t *i2c_bus)
  {
    if (i2c_files.count(bus_index) == 0)
      return false;

    if (i2c_open(i2c_bus, i2c_files[bus_index]) == 0)
      return true;

    // log initialization error
    string msg = "I2C device inititialization failure for bus #%d. "
        "Periphery library error message: \"%s\".";
    msg = format(msg, bus_index, i2c_errmsg(&i2c_bus));
    LogManager::get_instance().log_message(msg, LogManager::LogLevel::ERROR);

    return false;
  }

  /**
   * Initializes the SPI handle structure pointed to by spi_device for the given
   * bus and device indices if that spi device exists. Bus and device indices
   * are incremental and start at 0.
   *
   * Returns true if spi_device was successfully initialized; false otherwise.
   */
  bool init_spi_device(int bus_index, int device_index, spi_t *spi_device, SpiConf &settings)
  {
    std::pair<int, int> key;
    key.first = bus_index;
    key.second = device_index;

    if (spi_files.count(key) == 0)
      return false;

    int mode = get_spi_mode(settings.idle_level, settings.capture_edge);
    spi_bit_order_t bit_order = get_spi_bit_order(settings.endianness);

    int result = spi_open_advanced(spi_device, spi_files[key],
        mode, settings.max_speed, bit_order, settings.word_size, 0);
    if (result == 0)
      return true;

    // log initialization error
    string msg = "SPI device inititialization failure for device "
        "#%d on bus #%d. Periphery library error message: \"%s\".";
    msg = format(msg, device_index, bus_index, spi_errmsg(&spi_device));
    LogManager::get_instance().log_message(msg, LogManager::LogLevel::ERROR);

    returns false;
  }

  /**
   * Initializes the UART handle structure pointed to by uart_device for the
   * given device index if that uart device exists.  Device indices are
   * incremental and start at 0.
   *
   * The UART device will be configured for 8-bit word size, 1 stop bit, and
   * no hardware flow control (rts/cts).
   *
   * Returns true if uart_device was successfully initialized; false otherwise.
   */
  bool init_uart_device(int device_index, serial_t *uart_device, UartConf &settings)
  {
    if (uart_files.count(device_index) == 0)
      return false;

    int result = serial_open_advanced(uart_device, uart_files[device_index],
        settings.baudrate, 8, get_serial_parity(settings.parity), 1, settings.xonxoff, false);
    if (result == 0)
      return true;

    // log initialization error
    string msg = "UART device inititialization failure for device #%d. "
        "Periphery library error message: \"%s\".";
    msg = format(msg, device_index, serial_errmsg(&uart_device));
    LogManager::get_instance().log_message(msg, LogManager::LogLevel::ERROR);

    return false;
  }

  /**
   * Returns the number of i2c buses available.
   * This number is 1 greater than the largest i2c bus index.
   */
  int i2c_bus_count()
  {
    return i2c_buses;
  }

  /**
   * Returns the number of spi buses available.
   * This number is 1 greater than the largest spi bus index.
   */
  int spi_bus_count()
  {
    return spi_buses;
  }

  /**
   * Returns the number of spi devices available on a particular bus. If the
   * given bus index is out of the valid range then -1 is returned.
   * This number is 1 greater than the largest spi device index for the given
   * bus.
   */
  int spi_device_count(int bus_index)
  {
    if (bus_index >= spi_buses)
      return -1;
    return spi_devices[bus_index];
  }

  /**
   * Returns the number of uart buses available.
   * This number is 1 greater than the largest uart bus index.
   */
  int uart_bus_count()
  {
    return i2c_buses;
  }

};

#endif
