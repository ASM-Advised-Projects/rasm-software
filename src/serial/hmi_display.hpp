/**
 * Defines the HMIDisplay class.
 */

#ifndef HMI_DISPLAY_INCLUDED
#define HMI_DISPLAY_INCLUDED

#include "uart.hpp"
#include "serial_manage.hpp"
#include "configuration.hpp"

/**
 * gen4 HMI Display; UART; half-duplex data transfer.
 */
class HMIDisplay
{
private:
  UARTDevice *uart_device;

  HMIDisplay()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::SERIAL);

    int dev = configs->getInt("hmi_display.uartdev");
    bool obtained = SerialManager::get_instance().get_uart_device(dev, uart_device);
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
