/**
 * Defines the UCBoard class.
 */

#ifndef RASM2_PERIPHERY_UC_BOARD_HPP
#define RASM2_PERIPHERY_UC_BOARD_HPP

#include <array>
#include <string>
#include <cstring>

#include "c-periphery/serial.h"

//#include "rasm2/periphery/peripheral_accessor.h"
//#include "rasm2/periphery/periphery_config.h"
#include "rasm2/control/joint.hpp"
//#include "rasm2/configuration.hpp"

/**
 * This class represents a service layer on top of the UART communication port
 * connected to the microcontroller on the signal processing board (termed
 * UCBoard here as short for microcontroller board). The 'services' provided by
 * an instance of this class are the following:
 *   - ping
 *   - get battery voltage
 *   - get encoder output
 *   - get encoder outputs
 *   - set motor pwm level
 *   - set motor pwm levels
 *
 * Note that if any one of the joint motors is set to a non-zero level and
 * none of the joints is subsequently set within the next 100 milliseconds
 * then the microcontroller automatically sets all motor levels to 0. This is
 * a fail-safe meant to keep the joints from going out of range if there is an
 * inadvertent disconnect between the SBC and uC.
 *
 * Communication Protocol:
 *   Serial Protocol
 *     - UART
 *     - 115200 buadrate
 *     - 8 data bits
 *     - no parity
 *     - 1 stop bit
 *     - no software flow control (Xon/Xoff)
 *     - no hardware flow control (RTS/CTS)
 *
 *   Command List
 *     description        received by uC      sent by uC
 *     - - - - - - - -    - - - - - - - -     - - - - - - - - - -
 *     comm check         p                   a
 *     get batt voltage   b                   [0-9][0-9][0-9][0|1]  (little-endian)
 *     get enc output     e[0-5]              [0-9][0-9][0-9][0|1]  (little-endian)
 *     set motor level    m[0-5][frz][0-9][0-9] (big-endian)  none
 *
 *   Notes
 *     The [0-5] received by the uC after the e or m characters is a joint index.
 *       - The 0, 1, 2, 3, 4, and 5 indices represent the base, shoulder, elbow,
 *         wrist-yaw, wrist-pitch, and wrist-roll joints, respectively.
 *     For the little-endian values, the most significant digit is sent last.
 *     The 'a' response stands for acknowledgement.
 */
class UCBoard
{
private:
  serial_t *uartport;
  unsigned int timeout;
  char sendbuf[32];
  char recvbuf[32];

public:
  UCBoard(const UCBoard &) = delete;
  void operator=(const UCBoard &) = delete;

  /**
   * Constructs a new UCBoard instance that uses the given configured uart port
   * to communicate to the microcontroller with the given timeout value in
   * milliseconds when waiting for any response.
   */
  UCBoard(serial_t *port, unsigned int timeout)
  : uartport(port)
  , timeout(timeout)
  {
  }

  /**
   * Returns a pointer to the serial_t instance that represents the designated
   * uart port as specified in the configuration subsystem to use for
   * communicating to the microcontroller.
   */
  /*static serial_t * get_default_port()
  {
    Poco::AutoPtr<MapConfiguration> configs = ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::PERIPHERY);
    int uartbus = configs->getInt("uc_board.uartbus");

    serial_t *uartport;
    UartConf uartconf;  // leave defaults
    uartconf.baudrate = BaudRate::_115200;
    bool success = PeripheryAccess::get_instance().
        init_uart_device(uartbus, uartport, uartconf);
    if (!success)
    {
      throw Poco::RuntimeException("In UCBoard::UCBoard()\n"
      "Initialization failed for UART device number " + std::to_string(uartbus) + ".");
    }

    return uartport;
  }*/

  /*bool init_uart_port(serial_t uartport, std::string device_file, int baudrate)
  {
    return serial_open(uartport, device_file.c_str(), baudrate) >= 0;
  }*/

  /**
   * Closes this UCBoard's serial port which will subsequently make it
   * impossible for this instance to communicate with the microcontroller.
   */
  void close()
  {
    serial_close(uartport);
  }

  /**
   * Sets the round_trip_millis argument to the time in milliseconds that it
   * takes a ping command to be responded to. If a timeout occurs then
   * round_trip_millis will be set to -1.
   *
   * Returns true if the uC board is connected and able to communicate without
   * timing out; false otherwise.
   */
  bool ping(int &round_trip_millis)
  {
    // send a 'p' character
    const char *command = "p";
    serial_write(uartport, (const uint8_t*)command, strlen(command));
    serial_flush(uartport);

    // receive an 'a' character
    int bytes_read = serial_read(uartport, (uint8_t*)recvbuf, 1, timeout);
    if (bytes_read == 1)
      return recvbuf[0] == 'a';
    return false;
  }

  /**
   * Returns the output of the ADC reading the voltage divider attached to the
   * battery voltage lines. This value is in the range [0, 1023].
   */
  double get_battery_voltage()
  {
    const char *command = "b";
    serial_write(uartport, (const uint8_t*)command, strlen(command));
    serial_flush(uartport);

    int bytes_read = serial_read(uartport, (uint8_t*)recvbuf, 4, 50);
    if (bytes_read != 4)
      return false;

    double output = 1*(recvbuf[0]-'0') + 10*(recvbuf[1]-'0') +
                    100*(recvbuf[2]-'0') + 1000*(recvbuf[3]-'0');
    return output;
  }

  /**
   * Sets the output argument to the current value of the encoder for the
   * specified joint. The encoder value is a raw output from the ADC reading
   * that encoder and is within the range [0, 1023]. Returns true if the encoder
   * output was successfully read; false if not.
   */
  bool get_encoder_output(Joint joint, int &output)
  {
    const char *command = ("e" + std::to_string((int)joint)).c_str();
    serial_write(uartport, (const uint8_t*)command, strlen(command));
    serial_flush(uartport);

    int bytes_read = serial_read(uartport, (uint8_t*)recvbuf, 4, 50);
    if (bytes_read != 4)
      return false;

    output = 1*(recvbuf[0]-'0') + 10*(recvbuf[1]-'0') +
             100*(recvbuf[2]-'0') + 1000*(recvbuf[3]-'0');
    return true;
  }

  /**
   * Fills the encoutputs array with the current values of all six encoders.
   * These values are raw outputs from the ADCs reading the encoders and is are
   * within the range [0, 1023]. Returns true if all of the encoder outputs were
   * successfully read; false otherwise.
   *
   * The indices of the encpositions array correspond to the ordinal values of
   * the joints they represent in the Joint enumeration. More explicitly:
   *   0 -> BASE
   *   1 -> SHOULDER
   *   2 -> ELBOW
   *   3 -> WRIST_YAW
   *   4 -> WRIST_PITCH
   *   5 -> WRIST_ROLL
   */
  bool get_encoder_outputs(std::array<int, 6> &encoutputs)
  {
    encoutputs.fill(0);
    bool success;
    for (int j = 0; j <= 6; ++j)
    {
      success = get_encoder_output((Joint)j, encoutputs[j]);
      if (!success)
        return false;
    }
    return true;
  }

  /**
   * Sets the PWM level of the motor for the specified joint. The PWM signal is
   * a floating point value in the range [-99, 99]. The magnitude of this value
   * represents the duty cycle. The sign represents the motor direction where
   * positive is CCW. If the PWM level is outside of this range then its
   * magnitude will be reduced to 99.
   */
  void set_motor_pwm(Joint joint, float pwm)
  {
    if (pwm > 99) pwm = 99;
    if (pwm < -99) pwm = -99;

    std::string command = "m" + std::to_string((int)joint);
    command += pwm >= 0 ? "f" : "r";
    if (pwm < 0) pwm = -pwm;
    command += std::to_string(((int)pwm)/10);
    command += std::to_string(((int)pwm)%10);

    serial_write(uartport, (const uint8_t*)command.c_str(), command.size());
    serial_flush(uartport);
  }

  /**
   * Sets the PWM levels for all six motors. Each PWM signal is a floating
   * point value in the range [-99, 99]. The magnitude of this value represents
   * the duty cycle. The sign represents the motor direction where positive is
   * CCW. If a PWM level is outside of this range then its magnitude will be
   * reduced to 99.
   *
   * The indices of the pwmlevels array correspond to the ordinal values of
   * the joints they represent in the Joint enumeration. More explicitly:
   *   0 -> BASE
   *   1 -> SHOULDER
   *   2 -> ELBOW
   *   3 -> WRIST_YAW
   *   4 -> WRIST_PITCH
   *   5 -> WRIST_ROLL
   */
  void set_motor_pwms(const std::array<float, 6> &pwmlevels)
  {
    for (int j = 0; j <= 6; ++j)
      set_motor_pwm((Joint)j, pwmlevels[j]);
  }
};

#endif
