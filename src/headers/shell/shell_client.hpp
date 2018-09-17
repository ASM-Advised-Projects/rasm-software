/**
 * Defines the RasmShellClient class.
 */

#ifndef SHELL_CLIENT_INCLUDED
#define SHELL_CLIENT_INCLUDED

#include "shell_server.hpp"

#include <string>
#include <cctype>

#include <Poco/Thread.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/String.h>

typedef void (*data_callback_ptr)(char *, int);
typedef void (*notify_callback_ptr)();

using std::string;

/**
 * A client endpoint that connects and communicates with a running RASM shell
 * server.
 */
class RasmShellClient : Poco::Runnable
{
private:
  Poco::Net::StreamSocket socket;
  data_callback_ptr recvdata_callback;  // receive-data callback
  notify_callback_ptr recvend_callback;  // receive-end callback
  notify_callback_ptr seshquit_callback;  // session-quit callback
  char recvbuff[MAX_SHELL_SEND_SIZE];

  /**
   * Repeatedly passes all received messages from the shell server that
   * aren't receive-end or session-quit messages to the receive-data
   * callback. Receive-end messages prompt a call to the receive-end callback
   * while shession-quit messages prompt a call to the session-quit callback.
   */
  void forward_responses()
  {
    int recvlength;
    while (true)
    {
      recvlength = socket.receiveBytes(recvbuff, MAX_SHELL_SEND_SIZE);
      if (strcmp(recvbuff, RESPONSE_END_MSG.c_str()) == 0)
        (*recvend_callback)();
      else if (strcmp(recvbuff, SESSION_END_MSG.c_str()) == 0)
        (*seshquit_callback)();
      else
        (*recvdata_callback)(recvbuff, recvlength);
    }
  }

  void run()
  {
    forward_responses();
  }

public:
  /**
   * Sets the receive-data (data_cb), request-end (end_cb), and session-quit
   * (quit_cb) callbacks.
   */
  RasmShellClient(data_callback_ptr data_cb, notify_callback_ptr end_cb,
      notify_callback_ptr quit_cb)
  {
    recvdata_callback = data_cb;
    recvend_callback = end_cb;
    seshquit_callback = quit_cb;
  }

  /**
   * Binds this client, as a TCP socket, to the given IP address and port
   * number. It then connects to the shell server and initializes a session.
   * Depending on the success of the connection this method will return:
   *    0 - if the connection was successful
   *   -1 - if the server refused to grant a shell session
   *   -2 - if an exception was thrown
   *
   * This method must be called and return a 0 before the send_command method
   * is used.
   */
  int connect(string shellIP, unsigned int shellPort)
  {
    try
    {
      Poco::Net::SocketAddress localAddress(
          Poco::Net::AddressFamily::IPv4,
          shellIP,
          shellPort
      );
      socket.connect(localAddress);
      socket.sendBytes(SESSION_REQUEST_MSG.c_str(), SESSION_REQUEST_MSG.length());

      char response[SESSION_GRANT_MSG.length()];
      socket.receiveBytes(response, SESSION_GRANT_MSG.length());
      if (strcmp(response, SESSION_GRANT_MSG.c_str()) == 0)
      {
        Poco::Thread forwardingThread;
        forwardingThread.start(*this);
        return 0;
      }
      return -1;
    }
    catch (std::runtime_error err)
    {
      return -2;
    }
  }

  /**
   * Sends the given command string to the shell server. The command will
   * have all whitespace trimmed from the start and end. If the command
   * length exceeds MAX_SHELL_RECV_SIZE then it will be further trimmed down
   * to the max size by dropping the trailing characters.
   */
  void send_command(string command)
  {
    command = Poco::trim(command);
    if (command.length() > MAX_SHELL_RECV_SIZE)
      command = string(command.c_str(), MAX_SHELL_RECV_SIZE);
    socket.sendBytes(command.c_str(), command.length());
  }
};

#endif
