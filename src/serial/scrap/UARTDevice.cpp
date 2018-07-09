/**
 * UARTDevice.cpp
 */

#include "UARTDevice.h"
#include "../../platform/platform_defines.h"

#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

UARTDevice::UARTDevice(unsigned int dev, unsigned int baudrate) {
    this->dev = dev;
    this->baudrate = baudrate;
}

int UARTDevice::open() {
    const char *devfilename = (UART_PREFIX + std::to_string(dev)).c_str();
	fd = ::open(devfilename, O_RDWR | O_NOCTTY);
	if (fd < 0) return -1;

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

  return 0;
}

int UARTDevice::write(char *tx, int length) {
    if (::write(fd, tx, length) == -1)
		return -1;
	return 0;
}

int UARTDevice::read(char *rx, int length) {
    return ::read(fd, (void*)rx, length);
}

void UARTDevice::close() {
    ::close(fd);
}

UARTDevice::~UARTDevice() {
    this->close();
}
