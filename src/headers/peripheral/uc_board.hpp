/**
 * Defines the UCBoard (singleton) class.
 */

#ifndef UC_BOARD_INCLUDED
#define UC_BOARD_INCLUDED

#include "periphery/spi.h"
#include "periphery_access.hpp"
#include "periphery_config.hpp"
#include "configuration.hpp"

/**
 * AVR uController; SPI slave; motor driving / encoder reading / power
 *     managment board signal reading.
 */
class UCBoard
{
private:
  spi_t *spi_device;

  UCBoard()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::PERIPHERY);

    int bus = configs->getInt("uc_board.spibus");
    int dev = configs->getInt("uc_board.spidev");
    SpiConf settings;  // leave defaults

    bool initsuccess = PeripheryAccess::get_instance().
        init_spi_device(bus, dev, &spi_device, settings);
    if (!initsuccess)
    {
      // log & throw exception
    }
  }

public:
  UCBoard(const UCBoard &) = delete;
  void operator=(const UCBoard &) = delete;

  /**
   * Returns a reference to this singleton's instance.
   */
  static UCBoard& get_instance()
  {
    static UCBoard board;
    return board;
  }

  /**
   * UCBoard::get_instance().get_battery_voltage();
   */
  double get_battery_voltage()
  {

    return 0;
  }
};

#endif
