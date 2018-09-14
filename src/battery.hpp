/**
 * Implements the RASM's battery monitoring subsystem.
 * Defines the BatterySentinel class.
 */

#ifndef BATTERY_INCLUDED
#define BATTERY_INCLUDED

#include "configuration.hpp"
#include "peripheral/uc_board.hpp"

#include <Poco/Thread.h>
#include <Poco/Semaphore.h>

/**
 * This class is a singleton that monitors the battery's charge level and
 * provides basic queriable stats. The reason for this class being a singleton
 * is because it is inefficient for the UCBoard class to have multiple sentinels
 * retrieving the battery voltage. This in turn would needlessly use up
 * communication time with the signal processing board.
 */
class BatterySentinel : Poco::Runnable
{
private:
  Poco::Thread sentinelThread;
  Poco::Semaphore pauseSemaphore;

  double present_volts;
  double charge_percent;
  int remaining_time;
  bool charging;

  // TODO

  /**
   * Instantiates a new sentinel and starts it's processing loop in a newly
   * created thread.
   */
  BatterySentinel()
  : pauseSemaphore(0, 1)
  , present_volts(0)
  , charge_percent(0)
  , remaining_time(0)
  , charging(false)
  {
    // TODO

    // set the attributes of this runnable
    sentinelThread.setName("battery_sentinel");
    sentinelThread.setPriority(Poco::Thread::PRIO_LOW);

    // run this runnable in a separate thread
    sentinelThread.start(*this);
  }

  /**
   * Runs the processing loop of this sentinel.
   * Before the loop begins, relevant configurations are retrieved from the
   * configuration manager. The processing loop then begins and will only exit
   * if the destructor is called.
   */
  void run()
  {
    // get some settings
    std::string strvalue;
    ConfigurationManager &configs = ConfigurationManager::get_instance();

    // get the dead-level voltage
    configs.get_config_value(ConfigurationManager::Group::BATTERY, "deadvolts", strvalue);
    double deadvolts = Poco::NumberParser::parseFloat(strvalue);

    // get the monitoring period (millis)
    configs.get_config_value(ConfigurationManager::Group::BATTERY, "periodmillis", strvalue);
    int period = Poco::NumberParser::parseInt(strvalue);

    // get the UCBoard instance
    UCBoard &uc_board = UCBoard::get_instance();
    // for use via the uc_board.get_battery_voltage() method

    while (true)
    {
      // TODO

      // tryWait will return true if the destructor is ever called, otherwise
      // it will wait till the end of the period and return false
      if (pauseSemaphore.tryWait(period))
        break;
    }
  }

public:
  BatterySentinel(const BatterySentinel &) = delete;
  void operator=(const BatterySentinel &) = delete;

  /**
   * Returns a reference to this singleton's instance.
   */
  static BatterySentinel& get_instance()
  {
    static BatterySentinel manager;
    return manager;
  }

  ~BatterySentinel()
  {
    pauseSemaphore.set();  // exit the processing loop
  }

  /**
   * Returns the battery's present voltage in volts.
   */
  double battery_voltage()
  {
    return present_volts;
  }

  /**
   * Returns the battery's present level of charge as a percent value. The
   * percentage ranges from 0%, for completely discharged, to 100% for completely
   * charged. The percentage is directly proportional to the amount of time that
   * the battery can be used before it's fully discharged.
   */
  double battery_percent()
  {
    return charge_percent;
  }

  /**
   * Returns the time, in seconds, until the battery will be 'fully' discharged.
   * If the battery is charging at the moment, then the seconds returned is the
   * estimated length of time until the battery is fully charged.
   */
  int time_remaining()
  {
    return remaining_time;
  }

  /**
   * Returns true if the battery is being charged; false if not.
   */
  bool is_charging()
  {
    return charging;
  }

};

#endif
