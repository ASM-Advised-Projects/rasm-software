/**
 * Defines the UCBoard (singleton) class.
 */

#ifndef UC_BOARD_INCLUDED
#define UC_BOARD_INCLUDED

#include "periphery_access.hpp"
#include "periphery_config.h"
#include "../configuration.hpp"

#include "periphery/serial.h"

#include <array>

/**
 * AVR uController; UART slave; motor driving / encoder reading / power
 *     managment board signal reading.
 */
class UCBoard
{
private:
  serial_t *uart_port;

  UCBoard()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::PERIPHERY);
    int uartbus = configs->getInt("uc_board.uartbus");

    UartConf uartconf;  // leave defaults
    uartconf.baudrate = BaudRate::_115200;
    bool success = PeripheryAccess::get_instance().
        init_uart_device(uartbus, &uart_port, uartconf);
    if (!success)
    {
      throw Poco::RuntimeException("In UCBoard::UCBoard()\n"
      "Initialization failed for UART device number " + std::to_string(uartbus) + ".");
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
   *
   */
  bool is_pmb_on()
  {
    return false;
  }

  /**
   *
   */
  double get_battery_voltage()
  {
    return 0;
  }

  /**
   *
   */
  std::array<unsigned char, 6> get_md_errors()
  {
    std::array<unsigned char, 6> empty;
    return empty;
  }

  /**
   *
   */
  std::array<int, 6> get_encoder_positions()
  {
    std::array<int, 6> empty;
    return empty;
  }

  /**
   *
   */
  void set_motor_pwms(std::array<float, 6> pwmlevels)
  {
    return;
  }
};

#endif
