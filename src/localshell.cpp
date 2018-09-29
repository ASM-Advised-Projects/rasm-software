/**
 * A simple program that demonstrates usage of a RasmShellClient to provide
 * a frontend shell environment that communicates with a running RASM shell
 * server.
 *
 * To quit the session just type "quit" and press enter.
 */

#include "shell/shell_client.hpp"
#include <iostream>
#include <Poco/Semaphore.h>

using std::cout; using std::cin; using std::endl;
using std::string;

void run_session(RasmShellClient);
void message_data_callback(char *, int);
void response_end_callback();
void session_end_callback();

Poco::Semaphore response_end_sema(1);  // binary semaphore
bool quit = false;  // session quit flag

/**
 *
 */
int main(int argc, char **argv)
{
  //if (argc < 2)
  //  throw std::runtime_error("missing port number command line argument");
  //int port;
  //sscanf("%d", argv[1], &port);

  RasmShellClient client(
    &message_data_callback,
    &response_end_callback,
    &session_end_callback
  );

  cout << "connecting to shell server..." << endl;
  client.connect("127.0.0.1", SHELL_SERVER_PORT);

  cout << "connection established\nstarting a session" << endl;
  run_session(client);
}

/**
 *
 */
void run_session(RasmShellClient client)
{
  string command;
  while (true)
  {
    // wait till end of response for (a possible) previous command
    response_end_sema.wait();

    // exit if session is over
    if (quit) break;

    cout << ">> ";
    cin >> command;
    client.send_command(command);
  }
}

/**
 * Callback for shell data messages.
 * Directly prints all received character sequences to the standard output.
 */
void message_data_callback(char *response, int length)
{
  while (strcmp(response, RESPONSE_END_MSG.c_str()) != 0)
    cout << response;
}

/**
 * Callback for shell response end messages.
 * Releases a permit from binarysema.
 */
void response_end_callback()
{
  response_end_sema.set();
}

/**
 * Callback for shell quit response.
 * Sets the quit flag to true.
 */
void session_end_callback()
{
  quit = true;
}
