/**
 * Defines the UCBoard class.
 */

#ifndef UC_BOARD_INCLUDED
#define UC_BOARD_INCLUDED

#include "spi.hpp"
#include "serial_manage.hpp"
#include "configuration.hpp"

/**
 * AVR uController; SPI slave; motor driving / encoder reading / power
 *     managment board signal reading.
 */
class UCBoard
{
private:
  SPIDevice *spi_device;

  UCBoard()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::SERIAL);

    int bus = configs->getInt("uc_board.spibus");
    int dev = configs->getInt("uc_board.spidev");
    bool obtained = SerialManager::get_instance().get_spi_device(bus, dev, spi_device);
    if (!obtained)
    {
      // log & throw exception
    }


  }

public:
  UCBoard(const UCBoard &) = delete;
  void operator=(const UCBoard &) = delete;



};

#endif
