/**
 * Defines the HMIDisplay (singleton) class.
 */

#ifndef HMI_DISPLAY_INCLUDED
#define HMI_DISPLAY_INCLUDED

#include "periphery/serial.h"
#include "periphery_access.hpp"
#include "configuration.hpp"

/**
 * gen4 HMI Display; UART; half-duplex data transfer.
 */
class HMIDisplay
{
private:
  serial_t uart_device;

  HMIDisplay()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::PERIPHERY);

    int dev = configs->getInt("hmi_display.uartdev");
    UartConf settings;  // leave defaults

    bool initsuccess = SerialManager::get_instance().
        init_uart_device(dev, &uart_device, settings);
    if (!obtained)
    {
      // log & throw exception
    }
  }

public:
  HMIDisplay(const HMIDisplay &) = delete;
  void operator=(const HMIDisplay &) = delete;


};

#endif
