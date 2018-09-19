/**
 * Defines the RasmTime class.
 */

#include <Poco/Clock.h>

/**
 * Keeps track of the amount of time that has elapsed since startup. This
 * elapsed time can be statically queried.
 */
class RasmTime
{
private:
  static Poco::Clock startup_time;

  /**
   * Don't allow instance creation.
   */
  RasmTime()
  {
  }

public:
  static unsigned int current_time_millis()
  {
    return startup_time.elapsed();
  }

  static unsigned int current_time_seconds()
  {
    return 1000 * startup_time.elapsed();
  }
};
