/**
 * Implements the RASM's battery monitoring subsystem.
 * Defines the BatteryEstimator class.
 */

#ifndef BATTERY_INCLUDED
#define BATTERY_INCLUDED

#include "configuration.hpp"
#include "peripheral/uc_board.hpp"

#include <Poco/Thread.h>
#include <Poco/Semaphore.h>
#include <time.h>
#define CHARGING_INTERVAL_WAIT_TIME 5
#define MOTOR_INTERVAL_WAIT_TIME 5
#define CHARGING_RATE 0.05 // volts/second
#define DISCHARGE_RATE 0.05 // volts/second

/**
 * This class is a singleton that monitors the battery's charge level and
 * provides basic queryable stats. The reason for this class being a singleton
 * is because it is inefficient for the UCBoard class to have multiple estimators
 * retrieving the battery voltage. This in turn would needlessly use up
 * communication time with the signal processing board.
 */
class BatteryEstimator : Poco::Runnable
{
private:
  Poco::Thread estimatorThread;
  Poco::Semaphore pauseSemaphore;
  Controller &controller;

  double present_volts;
  double charge_percent;
  int remaining_time;
  bool charging;

  /**
   * Instantiates a new estimator and starts it's processing loop in a newly
   * created thread.
   */
  BatteryEstimator(Controller &ctl)
  : controller(ctl)
  , pauseSemaphore(0, 1)
  , present_volts(0)
  , charge_percent(0)
  , remaining_time(0)
  , charging(false)
  {
    // set the attributes of the estimator thread
    estimatorThread.setName("battery_estimator");
    estimatorThread.setPriority(Poco::Thread::PRIO_LOW);

    // start the estimator thread
    estimatorThread.start(*this);
  }

  /**
   * Runs the processing loop of this estimator.
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

    // get the UCBoard instance (for reading the battery voltage)
    UCBoard &uc_board = UCBoard::get_instance();

    double voltage;
    time_t previous_time_charging = time(0);
    time_t previous_time_motors = time(0);
    while (true)
    {
      // tryWait will return true if the destructor is ever called, otherwise
      // it will wait till the end of the period and return false
      if (pauseSemaphore.tryWait(period))
        break;

      // skip this iteration if the motors are currently running
      if (controller.motors_running())
      {
        continue;
      }
      else
      {
        previous_time_motors = time(0);
      }

      // get the present battery voltage
      voltage = uc_board.get_battery_voltage();

      // compute new battery state estimates
      double old_voltage = present_volts;
      present_volts = voltage;
      if(difftime(time(0), previous_time_charging) > CHARGING_INTERVAL_WAIT_TIME && difftime(time(0), previous_time_motors) > MOTOR_INTERVAL_WAIT_TIME) 
      { 
        charging = present_volts >= old_voltage; 
        previous_time_charging = time(0); 
      }
      if (charging)
      {
        double offset = 1;
      }
      else
      {
        double offset = 0;
      }
      if (present_volts - offset > 25.66)
      {
        charge_percent = 100;
      }
      else if (present_volts - offset > 25.44)
      {
        charge_percent = 90;
      }
      else if (present_volts - offset > 25.20)
      {
        charge_percent = 80;
      }
      else if (present_volts - offset > 24.94)
      {
        charge_percent = 70;
      }
      else if (present_volts - offset > 24.68)
      {
        charge_percent = 50;
      }
      else if (present_volts - offset > 24.12)
      {
        charge_percent = 40;
      }
      else if (present_volts - offset > 23.82)
      {
        charge_percent = 30;
      }
      else if (present_volts - offset > 23.52)
      {
        charge_percent = 20;
      }
      else if (present_volts - offset > 23.22)
      {
        charge_percent = 10;
      }
      else
      {
        charge_percent = 0;
      }
      if (charging)
      {
        remaining_time = (25.66-present_volts)/CHARGING_RATE;
      }
      else
      {
        remaining_time = (present_volts - 23.22)/DISCHARGE_RATE;
    }
  }

public:
  BatteryEstimator(const BatteryEstimator &) = delete;
  void operator=(const BatteryEstimator &) = delete;

  /**
   * Returns a reference to this singleton's instance.
   */
  static BatteryEstimator& get_instance()
  {
    static BatteryEstimator manager;
    return manager;
  }

  /**
   * Destructs this singleton by closing the processing thread.
   */
  ~BatteryEstimator()
  {
    pauseSemaphore.set();  // exit the processing loop
    estimatorThread.join();  // wait for processing thread to close

    // other destruction
    pauseSemaphore.~Semaphore();
    estimatorThread.~Thread();
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
