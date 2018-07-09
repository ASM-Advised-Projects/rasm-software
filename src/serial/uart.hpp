/**
 * Defines the UARTDevice class.
 */

#ifndef UART_INCLUDED
#define UART_INCLUDED

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

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
 *  - canonical receive mode (line-based input reading)
 */
class UARTDevice
{
private:
  int fd;  // uart device file descriptor

public:
  // the termios-allowed baud rates for an async comms port
  enum Baudrate
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
   * Constructs a uart device by opening and configuring the device file given
   * by filepath. This 'configuring' just happens at the file descriptor level.
   * The input and output baud rates are both set to baudrate.
   *
   * Throws an exception if the device file could not be opened or fully
   * configured.
   */
  UARTDevice(const std::string &filepath, Baudrate baudrate)
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

    // new termios struct (used to configure an async comms port settings)
    struct termios uartsettings;

    // initialize all settings to 0
    bzero(&uartsettings, sizeof(uartsettings));

    // set input & output speed to the given baudrate
    cfsetspeed(&uartsettings, (speed_t)baudrate);

    // input mode flags
    tcflag_t iflags = 0;
    iflags |= INPCK;  // enable input parity checking

    // output mode flags
    tcflag_t oflags = 0;

    // control mode flags
    tcflag_t cflags = 0;
    cflags |= CS8;  // 8-bit character size
    cflags |= CREAD;  // enable receiver
    cflags |= PARENB;  // enable output parity generation and input parity checking.
    cflags |= CLOCAL;  // ignore modem control lines

    // local mode flags
    tcflag_t lflags = 0;
    lflags |= ICANON;  // canonical mode

    // c_cc (special characters) settings
    // none
    //  - no additional EOF or EOL characters need to be defined
    //  - VTIME and VMIN only apply for noncanonical mode

    // apply the settings to the uart device file
    uartsettings.c_iflag = iflags;
    uartsettings.c_oflag = oflags;
    uartsettings.c_cflag = cflags;
    uartsettings.c_lflag = lflags;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &uartsettings);

    // check that all requested settings have been successfully set
    bzero(&uartsettings, sizeof(uartsettings));
    tcgetattr(fd, &uartsettings);
    bool success =
        uartsettings.c_iflag == iflags &&
        uartsettings.c_oflag == oflags &&
        uartsettings.c_cflag == cflags &&
        uartsettings.c_lflag == lflags &&
        cfgetispeed(&uartsettings) == (speed_t)baudrate &&
        cfgetospeed(&uartsettings) == (speed_t)baudrate;

    // throw exception if any settings could not be set
    if (!success)
    {
      // throw exception
    }
  }

  /**
   * Writes the given tx string to the output of this uart device.
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
   * Sets rx to a newline-delimited string read from this uart device. The
   * newline character will not be copied to rx. If the line length exceeds
   * 4096 characters then only the first 4096 characters will be read; the
   * rest are discarded.
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

#endif
