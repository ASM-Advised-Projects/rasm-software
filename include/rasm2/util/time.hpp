/**
 * Defines the Stopwatch class along with some time-related functions.
 */

#ifndef RASM2_UTIL_TIME_HPP
#define RASM2_UTIL_TIME_HPP

#include <chrono>
#include <array>
#include <stdexcept>

/**
 * Returns the current system time in microseconds.
 */
unsigned int current_time_micros()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/**
 * Returns the current system time in milliseconds.
 */
unsigned int current_time_millis()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/**
 * Returns the current system time in seconds.
 */
unsigned int current_time_seconds()
{
  return std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

/**
 * A simple class for clocking how long a section of code takes to execute.
 * This stopwatch is layered in the sense that multiple timers can be ran at
 * the same time. A timer is started by calling the start method. It is stopped
 * by calling the stop method. If the start method is called twice in a row,
 * then two timers are started. The stop method will stop the most recently
 * started timer. So the first call to the stop method will stop the 2nd timer
 * started. The 2nd call to the stop method will stop the 1st timer started.
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
 * one time, the start method can be called up to 5 more times than the stop
 * method. If the number of starts compared to stops become any more numerous
 * than that, or if the stop method is called more than the start method, an
 * exception is thrown.
 */
class Stopwatch
{
private:
  std::array<unsigned int, 5> start_times;
  int timers_running;
  unsigned int elapsed_time;

  unsigned int current_millis()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

public:
  Stopwatch(const Stopwatch &) = default;
  Stopwatch & operator=(const Stopwatch &) = default;

  /**
   * Constructs a new stopwatch with no timers running.
   */
  Stopwatch()
  : timers_running(0)
  , elapsed_time(0)
  {
    start_times.fill(0);
  }

  /**
   * Starts a new timer.
   * Throws a std::runtime_error exception if there are already 5 timers running.
   */
  void start()
  {
    if (timers_running == start_times.size())
    {
      throw std::runtime_error("Start cannot be called if there are already 5"
      " timers running.");
    }
    start_times[timers_running] = current_millis();
    timers_running++;
  }

  /**
   * Stops the most recently started timer.
   * Throws a std::runtime_error exception if there are no timers currently
   * running.
   */
  void stop()
  {
    if (timers_running == 0)
    {
      throw std::runtime_error("Stop cannot be called if there are no timers"
      " currently running.");
    }
    timers_running--;
    elapsed_time = current_millis() - start_times[timers_running];
  }

  /**
   * Returns the time elapsed in milliseconds for the timer that was stopped
   * last. In other words, returns the time between the last set of start-stop
   * calls.
   */
  unsigned int elapsed()
  {
    return elapsed_time;
  }
};

#endif  // UTIL_TIME_HPP
