/**
 * Defines the RasmTime and Stopwatch classes.
 */

#include <Poco/Clock.h>
#include <Poco/Exception.h>

#include <array>

/**
 * A singleton that keeps track of the amount of time that has elapsed since
 * program startup.
 */
class RasmTime
{
private:
  Poco::Clock start_time;

  RasmTime()
  {
  }

public:
  static RasmTime & get_instance()
  {
    static RasmTime rt;
    return rt;
  }

  /**
   * Returns the elapsed milliseconds since this singleton was initialized.
   */
  unsigned int current_time_millis()
  {
    return start_time.elapsed();
  }

  /**
   * Returns the elapsed seconds since this singleton was initialized.
   */
  unsigned int current_time_seconds()
  {
    return 1000 * start_time.elapsed();
  }
};

/**
 * A simple class for clocking how long a section of code takes to execute.
 * This stopwatch is layered in the sense that multiple timers can be ran at
 * the same time. A timer is started by calling the start method. It is stopped
 * by calling the stop method. If the start method is called twice in a row,
 * then two timers are started. The first call to the stop method will stop the
 * 2nd timer started. The 2nd call to the stop method will stop the 1st timer
 * started.
 *
 * Here's a more comprehensive example:
 * ...        // represents lines of other code
 * start()    // starts timer #1
 * ...
 * start()    // starts timer #2 - now 2 timers are running
 * ...
 * ...
 * stop()     // stops timer #2
 * elapsed()  // returns the time that timer #2 was running for
 * ...
 * start()    // starts timer #2 (starts back at 0 time elapsed; it doesn't
 *            // pick up where it was last stopped)
 * ...
 * elapsed()  // will return the same value it did before
 * stop()     // stops timer #2
 * elapsed()  // returns the time that timer #2 was just running for
 * stop()     // stops timer #1
 * elapsed()  // returns the time that timer #1 was running for
 *
 * A max of 5 timers can be running at the same time. In other words, at any
 * one time, the start method can have been called up to 5 more times than the
 * stop method. If the number of starts compared to stops become any more
 * numerous than that, or if the stop method is called more than the start
 * method, an exception is thrown.
 */
class Stopwatch
{
private:
  Poco::Clock clock;
  std::array<unsigned int,5> start_times;
  int current_timer;
  unsigned int elapsed_time;

public:
  /**
   * Constructs a new stopwatch with no timers running.
   */
  Stopwatch()
  : current_timer(-1)
  , elapsed_time(0)
  {
    start_times.fill(0);
  }

  ~Stopwatch()
  {
    clock.~Clock();
    start_times.~array();
  }

  /**
   * Starts a new timer.
   * Throws Poco::IllegalStateException if there are already 5 timers running.
   */
  void start()
  {
    if (current_timer == start_times.size())
    {
      throw Poco::IllegalStateException("In Stopwatch::start()\n"
      "Start cannot be called if there are already 5 timers running.");
    }
    start_times[++current_timer] = clock.elapsed();
  }

  /**
   * Stops the most recently started timer.
   * Throws Poco::IllegalStateException if there are no timers currently running.
   */
  void stop()
  {
    if (current_timer == -1)
    {
      throw Poco::IllegalStateException("In Stopwatch::stop()\n"
      "Stop cannot be called if there are no timers currently running.");
    }
    elapsed_time = clock.elapsed() - start_times[current_timer];
  }

  /**
   * Returns the time elapsed for the timer that was stopped last. In other
   * words, returns the time between the last set of start-stop calls.
   */
  unsigned int elapsed()
  {
    return elapsed_time;
  }
};
