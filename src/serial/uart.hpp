/**
 * Defines the UARTDevice class.
 */

#ifndef UART_INCLUDED
#define UART_INCLUDED

#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

/**
 *
 */
class UARTDevice
{
private:
  int fd;  // uart device file descriptor

public:
  UARTDevice(const std::string &filepath, int baudrate)
  {
    fd = ::open(filepath.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
      // throw exception
    }

    struct termios uart_port;
    bzero(&uart_port, sizeof(uart_port));

    uart_port.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    uart_port.c_iflag = IGNPAR | ICRNL;
    uart_port.c_oflag = 0;
    uart_port.c_lflag = 0;
    uart_port.c_cc[VTIME] = 0;
    uart_port.c_cc[VMIN]  = 1;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &uart_port);
  }

  /**
   *
   */
  virtual int write(const std::string &tx) {
    return ::write(fd, (char*)tx.c_str(), tx.length());
  }

  /**
   *
   */
  virtual int write(char *tx, int length) {
    return ::write(fd, tx, length);
  }

  /**
   *
   */
  virtual int read(char *rx, int length) {
      return ::read(fd, (void*)rx, length);
  }

  /**
   *
   */
  virtual int read(std::string &rx, int length) {
      char *rxptr;
      int result = ::read(fd, (void*)rxptr, length);
      rx = std::string(rxptr);
      return result;
  }

  /**
   *
   */
  virtual void close() {
      ::close(fd);
  }

  /**
   *
   */
  virtual ~UARTDevice() {
      this->close();
  }

};

#endif
