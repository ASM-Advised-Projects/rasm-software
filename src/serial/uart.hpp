/**
 * Defines the UARTDevice base class along with the UARTDeviceC (canonical read)
 * and UARTDeviceNC (non-canonical read) subclasses.
 */

#ifndef UART_INCLUDED
#define UART_INCLUDED

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

/**
 * The termios-allowed baud rates for async comms ports.
 */
enum BaudRate
{
  BR50 = B50,
  BR75 = B75,
  BR110 = B110,
  BR134 = B134,
  BR150 = B150,
  BR200 = B200,
  BR300 = B300,
  BR600 = B600,
  BR1200 = B1200,
  BR1800 = B1800,
  BR2400 = B2400,
  BR4800 = B4800,
  BR9600 = B9600,
  BR19200 = B19200,
  BR38400 = B38400,
  BR57600 = B57600,
  BR115200 = B115200,
  BR230400 = B230400
};

/**
 * Base class for opening a uart device file for reading and writing character-
 * based data.
 */
class UARTDevice
{
protected:
  int fd;  // uart device file descriptor

  /**
   *
   */
  struct termios get_base_settings(BaudRate baudrate)
  {
    // new termios struct (used to configure an async comms port settings)
    struct termios settings;

    // initialize all settings to 0
    bzero(&settings, sizeof(settings));

    // set input & output speed to the given baudrate
    cfsetspeed(&settings, (speed_t)baudrate);

    // input mode flags
    settings.c_iflag |= INPCK;  // enable input parity checking

    // output mode flags
    // none

    // control mode flags
    settings.c_cflag |= CS8;  // 8-bit character size
    settings.c_cflag |= CREAD;  // enable receiver
    settings.c_cflag |= PARENB;  // enable output parity generation and input parity checking.
    settings.c_cflag |= CLOCAL;  // ignore modem control lines

    // local mode flags
    // none
    // canonical mode: "c_lflag |= ICANON"

    // c_cc (special characters) settings
    // none
    // non-canonical mode: "c_cc[VMIN] = 0" and "c_cc[VTIME] = 0" (polling read)

    return settings;
  }

  /**
   *
   */
  bool apply_settings(struct termios &settings)
  {
    // apply the settings to the uart device file
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &settings);

    // check that all requested settings have been successfully set
    struct termios actual_settings;
    bzero(&actual_settings, sizeof(actual_settings));
    tcgetattr(fd, &actual_settings);
    bool success =
        actual_settings.c_iflag == settings.c_iflag &&
        actual_settings.c_oflag == settings.c_oflag &&
        actual_settings.c_cflag == settings.c_cflag &&
        actual_settings.c_lflag == settings.c_lflag &&
        actual_settings.c_cc[VMIN] == settings.c_cc[VMIN] &&
        actual_settings.c_cc[VTIME] == settings.c_cc[VTIME] &&
        cfgetispeed(&actual_settings) == cfgetispeed(&settings) &&
        cfgetospeed(&actual_settings) == cfgetospeed(&settings);

    return success;
  }

public:
  /**
   * Constructs a uart device by opening and configuring the device file given
   * by filepath. This 'configuring' just happens at the file descriptor level.
   * The input and output baud rates are both set to baudrate.
   *
   * Throws an exception if the device file could not be opened or fully
   * configured.
   */
  UARTDevice(const std::string &filepath)
  {
    /* Open the uart device file for reading and writing (O_RDWR).
     * O_NOCTTY keeps the device file from becoming this process's controlling
     * terminal since a uart device file is usually a /dev/tty character file.
     * This uart device file will be byte-by-byte so no need for O_SYNC. */
    fd = ::open(filepath.c_str(), O_RDWR | O_NOCTTY);

    // throw exception if device file couldn't be opened
    if (fd < 0)
    {
      // throw exception
    }
  }

  /**
   * Writes the given tx string to the output of this uart device.
   *
   * Returns true if the entire string was successfully written.
   * Returns false if an error occurred or if only part of the string could
   * be written.
   */
  virtual bool write(const std::string &tx)
  {
    if (tx.length() == 0)
      return true;
    int result = ::write(fd, (char*)tx.c_str(), tx.length());
    return result == tx.length();
  }

  /**
   * Closes the underlying character file for this uart device. All read/write
   * operations will fail after this method is called.
   */
  virtual void close()
  {
      ::close(fd);
  }

  /**
   * Destructs this uart device.
   * Closes this device's underlying character file.
   */
  virtual ~UARTDevice()
  {
      this->close();
  }
};


/**
 * Opens a uart device file for reading and writing character-based data at a
 * given baud rate. Binary data can also be written but the interface provided
 * by this class uses char sequences.
 *
 * This class has the following uart characteristics:
 *  - one start bit
 *  - 8-bit character
 *  - even parity
 *  - one stop bit
 *  - no hardware or software flow control
 *  - null byte (\0) on parity or frame error
 *  - canonical receive mode (line-based read)
 */
class UARTDeviceC : public UARTDevice
{
public:
  /**
   * Constructs and configures this device for a canonical read interface.
   */
  UARTDeviceC(const std::string &filepath, BaudRate baudrate)
  : UARTDevice(filepath)
  {
    struct termios settings = get_base_settings(baudrate);
    settings.c_lflag |= ICANON;

    bool success = apply_settings(settings);

    // throw exception if any settings could not be set
    if (!success)
    {
      // throw exception
    }
  }

  /**
   * Sets rx to a newline-delimited string read from this uart device. The
   * newline character will not be copied to rx. If the line length exceeds
   * 4096 characters then only the first 4096 characters will be read; the
   * rest are discarded.
   *
   * Returns true if a line was read into rx.
   * Returns false if an error occurred or if a line was not available to read
   * (in which case rx will remain unmodified).
   */
  virtual bool readline(std::string &rx)
  {
      char *rxptr;
      int result = ::read(fd, (void*)rxptr, 4096);

      // if error or nothing read
      if (result == -1 || result == 0)
        return false;

      // set rx and remove newlines
      rx = std::string(rxptr);
      if (*(rx.begin()) == '\n')
        rx.erase(rx.begin());
      if (*(rx.end()-1) == '\n')
        rx.erase(rx.end());

      return true;
  }
};


/**
 * Opens a uart device file for reading and writing character-based data at a
 * given baud rate. Binary data can also be written but the interface provided
 * by this class uses char sequences.
 *
 * This class has the following uart characteristics:
 *  - one start bit
 *  - 8-bit character
 *  - even parity
 *  - one stop bit
 *  - no hardware or software flow control
 *  - null byte (\0) on parity or frame error
 *  - non-canonical receive mode (non-delimited continuous read)
 */
class UARTDeviceNC : public UARTDevice
{
public:
  /**
   * Constructs and configures this device for a non-canonical read interface.
   */
  UARTDeviceNC(const std::string &filepath, BaudRate baudrate)
  : UARTDevice(filepath)
  {
    struct termios settings = get_base_settings(baudrate);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    bool success = apply_settings(settings);

    // throw exception if any settings could not be set
    if (!success)
    {
      // throw exception
    }
  }

  /**
   * This read method is non-blocking and is basically a polling read.
   *
   * Sets rx to a string read from this uart device. The max characters that
   * will be read is lesser of the given length and the number of characters
   * available. The max characters available is 4095. Any data that has arrived
   * at this device's input when 4095 chars are in the read buffer has been
   * discarded.
   *
   * Returns -1 if a read error occured; otherwise returns the number of
   * characters read from the read buffer into rx.
   */
  virtual int read(std::string &rx, int length)
  {
      char *rxptr;
      int result = ::read(fd, (void*)rxptr, 4096);
      rx = std::string(rxptr);
      return result;
  }
};

#endif
