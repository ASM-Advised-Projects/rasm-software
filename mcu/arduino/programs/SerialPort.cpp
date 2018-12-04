/**
 * This program runs two concurrent character pipes (one for input and one for
 * output) to the specified serial device at a specified baudrate. The serial
 * device file path is given as the first command line argument while the baudrate
 * is given as the second command line argument.
 *
 * Example usage:
 * $ ./SerialPort.cpp /dev/cu.usbmodem1421 115200
 */

#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <thread>
#include "serial.h"

using std::cerr;
using std::cout;
using std::endl;
using std::cin;
using std::string;

serial_t uart_port;
uint8_t send_buf[128];
uint8_t recv_buf[128];

bool closing = false;

void run_outgoing_pipe()
{
  uint8_t buf[128];
  string command;
  while (true)
  {
    cout << "waiting for command" << endl;
    cin >> command;
    if (command == "q")
      break;

    cout << "outgoing command" << endl;

    // send command string excluding the terminating null byte
    if (serial_write(&uart_port, (const uint8_t*)command.c_str(), sizeof(command.c_str()-1)) < 0) {
        cerr << "serial_write(): %s\n" << serial_errmsg(&uart_port) << endl;
        break;
    }
    serial_flush(&uart_port);
  }
}

void run_incoming_pipe()
{
  uint8_t buf[128];
  unsigned int *bytes_waiting;
  int bytes_read;
  while (true)
  {
    cout << "start poll" << endl;
    serial_poll(&uart_port, 1000);
    cout << "end poll" << endl;
    serial_input_waiting(&uart_port, bytes_waiting);
    if (*bytes_waiting > 0)
    {
      if ((bytes_read = serial_read(&uart_port, buf, *bytes_waiting, 50)) < 0) {
          cerr << "serial_read(): %s\n" << serial_errmsg(&uart_port) << endl;
          exit(1);
      }
      cout << "bytes read" << endl;
    }
  }
}

int main(int argc, char **argv)
{
  cout << "main - 1" << endl;
  if (argc != 3)
  {
    std::cerr << "Exactly two command line arguments are required." << endl;
    return 1;
  }

  cout << "main - 2" << endl;
  // open device with specified baudrate, and defaults of 8N1 with no flow control
  if (serial_open(&uart_port, argv[1], std::atoi(argv[2])) < 0) {
      cerr << "serial_open(): %s\n" << serial_errmsg(&uart_port) << endl;
      return 1;
  }

  cout << "main - 3" << endl;
  std::thread outpipe_thread(run_outgoing_pipe);
  std::thread inpipe_thread(run_incoming_pipe);

  cout << "main - 4" << endl;
  outpipe_thread.join();

  cout << "main - 5" << endl;
  closing = true;
  inpipe_thread.join();

  cout << "main - 6" << endl;
  serial_close(&uart_port);

  return 0;
}
