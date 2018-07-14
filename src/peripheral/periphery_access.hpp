/**
 * Defines the PeripheryAccess class.
 */

#ifndef PERIPHERY_ACCESS_INCLUDED
#define PERIPHERY_ACCESS_INCLUDED

#include "periphery/i2c.h"
#include "periphery/spi.h"
#include "periphery/serial.h"
#include "configuration.hpp"

#include "Poco/Format.h"

#include <map>

using std::string;
using Poco::format;

// spi mode(s)?
// spi speeds?
// uart parity?
// uart settings struct

/**
 * Allowed UART baud rates.
 */
enum UartBaudRate
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
 * This class is a singleton that provides access to a set of I2C buses, SPI
 * devices, and UART devices. Access is provided with contiguous bus/device
 * indices which correspond to i2c_t, spi_t, and serial_t instances.
 *
 * Note that a bus/device instance is only created when it is requested.
 *
 *
 */
class PeripheralManager
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
   * Constructs this manager using the PERIPHERAL configuration group.
   */
  SerialManager()
  {
    // get configuration group
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
      get_config_group(ConfigurationManager::Group::PERIPHERAL);

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
        std::pair<int, int> configkey2 = std::pair<int, int>(bus, device);
        spi_files[metakey] = configs->getString(configkey2);
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
  SerialManager(const SerialManager &) = delete;
  void operator=(const SerialManager &) = delete;

  /**
   * Returns this singleton's instance.
   */
  static SerialManager& get_instance()
  {
    static SerialManager instance;
    return instance;
  }

  /**
   * Initializes the I2C handle structure pointed to by i2c_bus for the given bus
   * index if the corresponding i2c bus exists. Bus indices are incremental and
   * start at 0.
   *
   * Returns true if i2c_bus was successfully initialized; false otherwise.
   */
  bool get_i2c_bus(int bus_index, i2c_t *i2c_bus)
  {
    if (i2c_files.count(bus_index) == 0)
      return false;

    if (i2c_open(i2c_bus, i2c_files[bus_index]) == 0)
      return true;

    // TODO -- log error -- log("i2c init failure: " + i2c_errmsg(&i2c_bus))
  }

  /**
   * Initializes the SPI handle structure pointed to by spi_device for the given
   * bus and device indices if that spi device exists. Bus and device indices
   * are incremental and start at 0.
   *
   * Returns true if spi_device was successfully initialized; false otherwise.
   */
  bool get_spi_device(int bus_index, int device_index, spi_t *spi_device)
  {
    std::pair<int, int> key;
    key.first = bus_index;
    key.second = device_index;

    if (spi_files.count(key) == 0)
      return false;

    if (spi_open(spi_device, spi_files[key]) == 0)  // TODO -- add mode and speed
      return true;

    // TODO -- log error -- log("spi init failure: " + spi_errmsg(&spi_device))
  }

  /**
   * Initializes the UART handle structure pointed to by uart_device for the
   * given device index if that uart device exists.  Device indices are
   * incremental and start at 0.
   *
   * Returns true if uart_device was successfully initialized; false otherwise.
   */
  bool get_uart_device(int device_index, serial_t *uart_device)
  {
    if (uart_files.count(device_index) == 0)
      return false;

    if (serial_open(uart_device, uart_files[device_index]))  // TODO -- add settings
      return true;

    // TODO -- log error -- log("uart init failure: " + serial_errmsg(&uart_device))
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
