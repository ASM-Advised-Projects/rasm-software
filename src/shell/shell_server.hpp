/**
 * Defines the ShellSession and RasmShellServer classes.
 */

#ifndef RASM_SHELL_INCLUDED
#define RASM_SHELL_INCLUDED

#include <string.h>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Thread.h>
#include <Poco/Runnable.h>

#define SHELL_SERVER_PORT 5000
#define MAX_SHELLS 5
#define MAX_SHELL_RECV_SIZE 256
#define MAX_SHELL_SEND_SIZE 1024

#define SESSION_REQUEST_MSG string("_session_request_")
#define SESSION_GRANT_MSG string("_session_granted_")
#define SESSION_END_MSG string("_session_end_")
#define RESPONSE_END_MSG string("_response_end_")

#define THREAD_PRIORITY Poco::Thread::Priority::PRIO_LOW

using std::string;

/**
 * Holds a collection of pointers to instances that provide the data
 * needed by the rasm shell.
 */
struct DataComponents
{

} backend;

void set_backend(DataComponents components)
{
  backend = components;
}

/**
 * Only for internal use by the RasmMultiShell class.
 *
 * Provides a single shell context that handles a session with a client until
 * either the client quits the session or the close method is called. Session
 * initialization and command parsing is encapsulated in this class.
 */
class ShellSession : Poco::Runnable
{
private:
  Poco::Net::StreamSocket socket;  // connected tcp socket
  char recvbuff[MAX_SHELL_RECV_SIZE];  // receive buffer
  char sendbuff[MAX_SHELL_SEND_SIZE];  // send buffer
  bool ongoing;

  void run()
  {
    ongoing = true;

    try {
      int recvlength = socket.receiveBytes(recvbuff, MAX_SHELL_RECV_SIZE);
      if (strcmp(recvbuff, SESSION_REQUEST_MSG.c_str()) == 0)
      {
        socket.sendBytes(SESSION_GRANT_MSG.c_str(), SESSION_GRANT_MSG.length());
        while (true)
        {
          // wait for client command
          recvlength = socket.receiveBytes(recvbuff, MAX_SHELL_RECV_SIZE);

          process_command(recvlength);

          // send response-end indication
          socket.sendBytes(RESPONSE_END_MSG.c_str(), RESPONSE_END_MSG.length());
        }
      }
      else
      {
        socket.close();
      }
    }
    catch (std::exception e)
    {
      // likely caused by close method closing the socket; do nothing
    }

    ongoing = false;
  }

  void process_command(int cmdlength)
  {
    // use a constructor-initialized parse tree
    // hash map of string cmds to method handles
    // organize by subsystem
    // control, http, peripheral, shell?, vision, battery, log, manage (for mode)
    // handling of -h and --help commands
    //   lists subsystem-specific command structure w/valid tokens
    // invalid commands
  }

  void close()
  {
    if (ongoing)
    {
      string message = "closing due to backend circumstances";
      socket.sendBytes(message.c_str(), message.length());
    }
    socket.close();
  }

public:
  ShellSession()
  {
    ongoing = false;
  }

  ~ShellSession()
  {
    close();
    socket.~StreamSocket();
  }

  void start(Poco::Net::StreamSocket ss)
  {
    socket = ss;
    Poco::Thread shellThread;
    shellThread.setPriority(THREAD_PRIORITY);
    shellThread.start(*this);
  }

  bool is_ongoing()
  {
    return ongoing;
  }
};

/**
 * Runs up to a configurable amount of shell backends on a set port number
 * with a TCP socket.
 */
class RasmShellServer : Poco::Runnable
{
private:
  Poco::Net::ServerSocket server_socket;  // TCP server socket
  Poco::Net::StreamSocket connected_socket;  // TCP connection socket
  char recvbuff[MAX_SHELL_RECV_SIZE];  // receive buffer
  char sendbuff[MAX_SHELL_SEND_SIZE];  // send buffer
  ShellSession sessions[MAX_SHELLS];

  /**
   * Repeatedly listens for session requests and grants each of them a shell
   * session with the restriction that there can only be a max of 5 shells
   * at any one time.
   */
  void listen_for_connections()
  {
    try
    {
      int recvlen;
      while (true)
      {
        connected_socket = server_socket.acceptConnection();
        connected_socket.setNoDelay(true);
        for (int i = 0; i < MAX_SHELLS; i++)
        {
          if (!sessions[i].is_ongoing())
            sessions[i].start(connected_socket);
        }
      }
    } catch (std::runtime_error err)
    {
      // likely caused by destructor closing the server socket; do nothing
    }
  }

  void run()
  {
    listen_for_connections();
  }

public:
  /**
   * Creates the TCP server socket on the configured port and starts listening
   * for client endpoints to connect to.
   */
  RasmShellServer()
  {
    Poco::Net::SocketAddress localAddress(
        Poco::Net::AddressFamily::IPv4,
        "127.0.0.1",
        SHELL_SERVER_PORT
    );
    server_socket.bind(localAddress);

    Poco::Thread listenThread;
    listenThread.setPriority(THREAD_PRIORITY);
    listenThread.start(*this);
  }

  /**
   * Ends all ongoing sessions and stops listening for connections. Closing
   * messages are sent prior to shutting down any ongoing sessions.
   */
  ~RasmShellServer()
  {
    for (int i = 0; i < MAX_SHELLS; i++)
      sessions[i].~ShellSession();
    server_socket.close();
    server_socket.~ServerSocket();
  }
};

#endif
