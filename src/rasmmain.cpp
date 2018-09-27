/**
 *
 */

#include "configuration.hpp"
#include "logging.hpp"
#include "shell/shell_server.hpp"
#include "http/http_server.hpp"
//#include "control/control.hpp"
#include "peripheral/uc_board.hpp"
#include "battery.hpp"

#include <Poco/Semaphore.h>
#include <Poco/Process.h>


/**
 * Handles the startup and shutdown of all RASM subsystems.
 */
class RasmManager
{
private:
  bool running;
  Poco::Semaphore shutdown_sema;

  RasmManager() : shutdown_sema(1)
  {
    running = false;
  }

  /**
   * The callback for when the RASM needs to shutdown. This method should be
   * registered by the battery sentinel and by the shell server so either
   * of them can signal when it's time to shutdown.
   */
  void shutdown_callback()
  {
    shutdown_sema.set();  // release shutdown semaphore permit
  }

public:
  RasmManager(const RasmManager &) = delete;
  void operator=(const RasmManager &) = delete;

  /**
   * Initializes all RASM subsystems, starts the controller, waits for a shutdown
   * shutdown signal from either the UCBoard or control subsystems, then shuts
   * down all subsystems.
   */
  int run()
  {
    if (running)
      return -1;
    running = true;

    // initialize all singletons and top-level non-singleton subsystems
    ConfigurationManager &config_manager = ConfigurationManager::get_instance();
    LogManager &log_manager = LoggingManager::getInstance();
    UCBoard &uc_board = UCBoard::get_instance();
    BatterySentinel &battery_sentinel
    RasmShellServer shell_server;
    RasmHttpServer http_server;
    Controller controller;
    // vision system is not top-level

    controller.start();

    // register this class's shutdown callback with the UCBoard and battery sentinel
    UCBoard.get_instance().register_shutdown_callback(shutdown_callback);
    controller.register_shutdown_callback(shutdown_callback);

    // wait for call to shutdown_callback
    shutdown_sema.wait();

    // close all subsystems in logical order (may not be the same as the
    // reverse of the order they were created in)
    shell_server.~RasmShellServer();
    http_server.~RasmHttpServer();
    controller.~Controller();
    battery_sentinel.~BatterySentinel();
    uc_board.~UCBoard();
    config_manager.~ConfigurationManager();
    log_manager.~LoggingManager();
    // logging subsystem should come last since any destructor may attempt to
    // log something
  }

  /**
   * Returns a reference to this singleton's instance.
   */
  static RasmManager& get_instance()
  {
    static RasmManager manager;
    return manager;
  }
};


void start_shutdown()
{
  std::string command = "shutdown";
  std::vector<std::string> args;
  args[0] = "-P";
  args[1] = "now";
  Poco::Process::launch(command, args);
}


int main(int argc, char **argv)
{
  // pass the root working directory to the configuration manager
  if (argc < 2)
    throw std::runtime_error("root working directory must be specified");
  ConfigurationManager::rwd = argv[1];

  // run the RASM manager
  try
  {
    RasmManager::get_instance().run();
  }
  catch (std::exception err)
  {
    start_shutdown();
    return 1;
  }

  start_shutdown();
  return 0;
}

#endif
