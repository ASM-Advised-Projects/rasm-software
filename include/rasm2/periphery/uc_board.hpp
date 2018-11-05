/**
 * Defines the UCBoard class.
 */

#ifndef RASM2_PERIPHERY_UC_BOARD_HPP
#define RASM2_PERIPHERY_UC_BOARD_HPP

#include <array>

#include "periphery/serial.h"

#include "periphery_access.hpp"
#include "periphery_config.h"
#include "configuration.hpp"

/**
 * This class acts as the service layer on top of the UART communication port
 * connected to the microcontroller on the signal processing board (termed
 * UCBoard here as short for microcontroller board). The 'services' provided
 * by an instance of this class are the following:
 *   - is connected
 *   - is power on
 *   - get battery voltage
 *   - get encoder outputs
 *   - set motor pwm levels
 * The microcontroller is communicated with as a stateless synchronous-response
 * peripheral.
 */
class UCBoard
{
private:
  serial_t *uartport;

public:
  UCBoard(const UCBoard &) = delete;
  void operator=(const UCBoard &) = delete;

  /**
   * Constructs a new UCBoard instance that uses the given configured uart port to
   * communicate to the microcontroller.
   */
  UCBoard(serial_t *port)
  : uartport(port)
  {
  }

  /**
   * Returns a pointer to the serial_t instance that represents the designated
   * uart port as specified in the configuration subsystem to use for
   * communicating to the microcontroller.
   */
  static serial_t * get_default_port()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::PERIPHERY);
    int uartbus = configs->getInt("uc_board.uartbus");

    serial_t *uartport;
    UartConf uartconf;  // leave defaults
    uartconf.baudrate = BaudRate::_115200;
    bool success = PeripheryAccess::get_instance().
        init_uart_device(uartbus, &port, uartconf);
    if (!success)
    {
      throw Poco::RuntimeException("In UCBoard::UCBoard()\n"
      "Initialization failed for UART device number " + std::to_string(uartbus) + ".");
    }

    return uartport;
  }

  /**
   * Returns true if the uC board is connected and communicating properly.
   *
   * Specification for signal processing microcontroller (uC):
   *   - communication over UART port at baud rate of 115200
   *   - data sent to uC: p
   *   - data received from uC: a
   */
  bool is_connected()
  {
    return false;
  }

  /**
   * Returns true if the power management board is still switched on; false if
   * not.
   *
   * Specification for signal processing microcontroller (uC):
   *   - communication over UART port at baud rate of 115200
   *   - data sent to uC: o
   *   - data received from uC: [0-1]
   */
  bool is_pmb_on()
  {
    return false;
  }

  /**
   * Returns the current battery voltage with a resolution of 0.029.
   * (resolution = division_factor * V_adc * 2^-bits = 6*5*2^-10 = 0.029)
   *
   * Specification for signal processing microcontroller (uC):
   *   - communication over UART port at baud rate of 115200
   *   - data sent to uC: b
   *   - data received from uC: [0-1][0-9][0-9][0-9] (little-endian)
   */
  double get_battery_voltage()
  {
    return 0;
  }

  /**
   * Returns the values of all six encoders. These values are plain outputs
   * from the ADCs reading the encoder outputs.
   *
   * Specification for signal processing microcontroller (uC):
   *   - communication over UART port at baud rate of 115200
   *   - data sent to uC: e[0-5]
   *   - data received from uC: [0-1][0-9][0-9][0-9] (little-endian)
   */
  std::array<int, 6> get_encoder_outputs()
  {
    std::array<int, 6> empty;
    return empty;
  }

  /**
   * Sets the PWM levels for all six motors. Each PWM signal is a floating
   * point value in the range [-100, 100]. The magnitude of this value
   * represents the duty cycle while the sign is the motor direction. If a PWM
   * level is outside of this range then it will be changed to the range
   * boundary that is closest.
   *
   * Specification for signal processing microcontroller (uC):
   *   - communication over UART port at baud rate of 115200
   *   - data sent to uC: m[0-5][frz][0-9][0-9][0-9] (big-endian)
   *   - data received from uC: a
   */
  bool set_motor_pwms(std::array<float, 6> pwmlevels)
  {
    return;
  }
};

#endif
