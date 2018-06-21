/**
 * Defines the RasmManager class and the main method.
 */

#ifndef MANAGE_INCLUDED
#define MANAGE_INCLUDED

#include "configuration.hpp"
#include "logging.hpp"
#include "shell_server.hpp"
#include "http_server.hpp"
#include "battery.hpp"
#include "control.hpp"

#include <Poco/Semaphore.h>


/**
 * Handles the startup and shutdown of all RASM subsystems.
 */
class RasmManager
{
public:
  RasmManager(const RasmManager &) = delete;
  void operator=(const RasmManager &) = delete;

  /**
   * Initializes all RASM subsystems, starts the controller, waits for a
   * shutdown signal from the shell server or battery sentinel, then closes
   * all subsystems.
   */
  int run()
  {
    if (running)
      return -1;
    running = true;

    // initialize all RASM subsystems (includes initializing all
    // lazily-initialized singletons)
    config::ConfigurationManager &config_manager = config::ConfigurationManager::get_instance();
    log::LoggingManager &log_manager = log::LoggingManager::getInstance();
    energy::BatterySentinel &batt_sentinel = energy::BatterySentinel::getInstance();
    shell::RasmShellServer shell_server;
    web::RasmHttpServer http_server;
    control::Controller controller;
    controller.start();

    // register this classes shutdown callback with the battery sentinel
    batt_sentinel.add_shutdown_callback(shutdown_callback);

    // wait for call to shutdown_callback
    shutdown_sema.wait();

    // close/destroy all subsystems in logical order (may not be the same
    // as the reverse of the order they were created in)
    shell_server.~RasmShellServer();
    http_server.~RasmHttpServer();
    controller.~Controller();
    batt_sentinel.~BatterySentinel();
    config_manager.~ConfigurationManager();
    log_manager.~LoggingManager();
  }

  /**
   * Returns a reference to this singleton's instance.
   */
  static RasmManager& get_instance()
  {
    static RasmManager manager;
    return manager;
  }

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
    shutdown_sema.set();  // release shutdown_sema permit
  }
};


int main(int argc, char **argv)
{
  // pass the root working directory to a config namespace global
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
    throw;
  }

  return 0;
}

#endif
