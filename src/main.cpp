/**
 * Defines the RasmManager class and main method.
 */

#include <Poco/Semaphore.h>
#include <Poco/Process.h>

#include "rasm2/configuration.hpp"
#include "rasm2/logging.hpp"
#include "rasm2/shell/shell_server.hpp"
#include "rasm2/http/http_server.hpp"
#include "rasm2/control/control.hpp"
#include "rasm2/periphery/uc_board.hpp"
#include "rasm2/battery.hpp"

/**
 * Handles the startup and shutdown of all RASM subsystems.
 */
class RasmManager
{
private:
  bool running;

  RasmManager()
  : running(false)
  {
  }

public:
  RasmManager(const RasmManager &) = delete;
  void operator=(const RasmManager &) = delete;

  /**
   * Returns a reference to this singleton's instance.
   */
  static RasmManager & get_instance()
  {
    static RasmManager manager;
    return manager;
  }

  /**
   * Initializes all RASM subsystems, starts the controller, waits for a
   * shutdown signal from either the periphery or control subsystems, then shuts
   * down all subsystems.
   */
  void run()
  {
    if (running)
      return;
    running = true;

    // initialize all subsystems other than vision (handled by control subsystem)
    ConfigurationManager &config_manager = ConfigurationManager::get_instance();
    LogManager &log_manager = LoggingManager::getInstance();
    UCBoard uc_board(UCBoard::get_default_port());
    BatterySentinel battery_estimator(
        std::bind(&UC_Board::get_battery_voltage, uc_board));
    Controller controller(uc_board, battery_estimator);
    //RasmHttpServer http_server;
    //RasmShellServer shell_server;

    // run the controller
    controller.control();
  }
};

/**
 *
 */
int main(int argc, char **argv)
{
  // shutdown command and its arguments
  std::string command = "shutdown";
  std::vector<std::string> args;
  args.push_back("-P");
  args.push_back("now");

  // run the RASM manager
  try
  {
    RasmManager::get_instance().run();
  }
  catch (std::exception err)
  {
    Poco::Process::launch(command, args);
    return 1;
  }

  Poco::Process::launch(command, args);
  return 0;
}
