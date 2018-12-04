
#include <iostream>
#include <string>

#include "c-periphery/serial.h"

using std::string;
using std::cout;
using std::cin;
using std::endl;

int main(int argc, char **argv)
{
  serial_t uartport1;

  std::string device_file_1 = "/dev/ttyO1";

  int baudrate = 115200;

  int result1 = serial_open(&uartport1, device_file_1.c_str(), baudrate);
  cout << "uart port result: " << result1 << endl;

  char sendbuf[10];
  sendbuf[0] = 'p';
  int sendlen = 1;

  char recvbuf[10];
  int recvlen = 1;

  while (true)
  {
    serial_write(&uartport1, (const uint8_t *)sendbuf, sendlen);
    serial_flush(&uartport1);

    recvbuf[0] = 'n';
    int bytesread = serial_read(&uartport2, (uint8_t *)recvbuf, recvlen, 100);
    cout << bytesread << ", " << recvbuf[0] << endl;
  }
}
