/**
 * Defines the SerialManager class.
 */

#ifndef SERIAL_MANAGE_INCLUDED
#define SERIAL_MANAGE_INCLUDED

#include "i2c.hpp"
#include "spi.hpp"
#include "uart.hpp"
#include "configuration.hpp"

#include "Poco/Format.h"

#include <map>

using Poco::format;

struct I2CBusMeta
{
  string filepath;
  bool retrieved = false;
};

struct SPIDevMeta
{
  string filepath;
  bool retrieved = false;
};

struct UARTDevMeta
{
  string filepath;
  bool retrieved = false;
};


/**
 * This class is a singleton that manages access to a set of I2C buses, SPI
 * devices, and UART devices. Access to the actual buses/devices is achieved
 * with instances of the I2CBus, SPIDevice, and UARTDevice classes.
 *
 * If this singleton is all that is used to get these bus/device instances then
 * it is guaranteed that only one instance will exist per bus/device. Moreover,
 * it is also guaranteed that each instance is only retrieved once. This prevents
 * two separate parts of a software system from inadvertently using the same
 * bus/device.
 *
 * Note that a bus/device instance is only created when it is first requested.
 */
class SerialManager
{
private:
  // bus index to i2c bus meta-data struct mapping
  std::map<int, I2CBusMeta> i2c_metas;

  // bus and device index to spi device meta-data struct mapping
  std::map<std::pair<int, int>, SPIDevMeta> spi_metas;

  // device index to uart device meta-data struct mapping
  std::map<int, UARTDevMeta> uart_metas;

  int i2c_buses;     // number of i2c buses
  int spi_buses;     // number of spi buses
  int *spi_devices;  // number of spi devices for each bus
  int uart_devices;  // number of uart devices

  /**
   * Constructs this manager using the SERIAL configuration group.
   */
  SerialManager()
  {
    // get configuration group
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
      get_config_group(ConfigurationManager::Group::SERIAL);

    string configkey;

    // populate i2c meta map
    i2c_buses = configs->getInt("i2c.bus_count");
    for (int bus = 0; bus < i2c_buses; bus++)
    {
      configkey = format("i2c.bus%d_filepath", bus);
      i2c_metas[bus].filepath = configs->getString(configkey);
    }

    // populate spi meta map
    spi_buses = configs->getInt("spi.bus_count");
    spi_devices = new int[spi_buses];
    for (int bus = 0; bus < spi_buses; bus++)
    {
      configkey = format("spi.bus%d.device_count", bus);
      spi_devices[bus] = configs->getInt(configkey);
      for (int device = 0; device < spi_devices[bus]; device++)
      {
        configkey = format("spi.bus%d.device%d_filepath", bus, device);
        std::pair<int, int> metakey = std::pair<int, int>(bus, device);
        spi_metas[metakey].filepath = configs->getString(configkey);
      }
    }

    // populate uart meta map
    uart_devices = configs->getInt("uart.device_count");
    for (int device = 0; device < uart_devices; device++)
    {
      configkey = format("uart.device%d_filepath", device);
      uart_metas[device].filepath = configs->getString(configkey);
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
   * Assigns the given I2CBus pointer to a valid instance for the given bus
   * index if:
   *  - that i2c bus exists; and
   *  - this method hasn't been called yet with the same bus index
   * Bus indices are incremental and start at 0.
   *
   * Returns true if the I2CBus pointer was set to a valid instance; false
   * otherwise (in which case the pointer will remain unmodified).
   */
  bool get_i2c_bus(int bus_index, I2CBus *i2cBusPtr)
  {
    if (i2c_metas.count(bus_index) == 0)
      return false;

    if (i2c_metas[bus_index].retrieved)
      return false;

    i2cBusPtr = new I2CBus(i2c_metas[bus_index].filepath);
    i2c_metas[bus_index].retrieved = true;
    return true;
  }

  /**
   * Assigns the given SPIDevice pointer to a valid instance for the given bus
   * and device indices if:
   *  - that spi device exists; and
   *  - this method hasn't been called yet with those same two indices
   * Bus and device indices are incremental and start at 0.
   *
   * Returns true if the SPIDevice pointer was set to a valid instance; false
   * otherwise (in which case the pointer will remain unmodified).
   */
  bool get_spi_device(int bus_index, int device_index, SPIDevice *spiDevPtr)
  {
    std::pair<int, int> key;
    key.first = bus_index;
    key.second = device_index;

    if (spi_metas.count(key) == 0)
      return false;

    if (spi_metas[key].retrieved)
      return false;

    spiDevPtr = new SPIDevice(spi_metas[key].filepath);
    spi_metas[key].retrieved = true;
    return true;
  }

  /**
   * Assigns the given UARTDevice pointer to a valid instance for the given
   * device index if:
   *  - that uart device exists; and
   *  - this method hasn't been called yet with the same device
   * Device indices are incremental and start at 0.
   *
   * Returns true if the UARTDevice pointer was set to a valid instance; false
   * otherwise (in which case the pointer will remain unmodified).
   */
  bool get_uart_device(int device_index, UARTDevice *uartDevPtr)
  {
    if (uart_metas.count(device_index) == 0)
      return false;

    if (uart_metas[device_index].retrieved)
      return false;

    uartDevPtr = new UARTDevice(uart_metas[device_index].filepath);
    uart_metas[device_index].retrieved = true;
    return true;
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
