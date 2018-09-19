/**
 * Implements the RASM's battery monitoring subsystem.
 * Defines the BatteryEstimator class.
 */

#ifndef BATTERY_INCLUDED
#define BATTERY_INCLUDED

#include "time.hpp"
#include "configuration.hpp"
#include "control/signal_processing.hpp"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>
#include <Poco/Util/TimerTask.h>
#include <Poco/Util/TimerTaskAdapter.h>

#include <vector>

using std::string;


/**
 * An instance of this class periodically reads a battery's voltage, estimates
 * it's level of charge, and provides basic queryable stats.
 */
class BatteryEstimator : Poco::Runnable
{
private:
  /**
   * This struct holds a battery-level data point containing a time, voltage,
   * and voltage slope.
   */
  struct BatteryDataRecord
  {
    unsigned int seconds;
    double voltage;
    double voltage_slope;
  };

  double (*get_voltage)();
  bool reading_enabled;
  Poco::Util::Timer timer;

  RealTimeLTIFilter *voltage_filter;
  RealTimeDifferentiator voltage_diff;

  std::vector<BatteryDataRecord> charge_particles;
  std::vector<BatteryDataRecord> discharge_particles;
  int particle_trials;
  double voltage_error_weight;
  double slope_error_weight;
  int dead_volts;

  double battery_voltage_;
  double battery_percent_;
  int time_remaining_;
  bool charging_;

  /**
   * Parses the given string into a vector of numbers. The numbers in the string
   * are expected to be comma-delimited.
   */
  std::vector<double> parse_number_list(const string &num_list)
  {
    Poco::StringTokenizer splitter(num_list, ",");
    std::vector<double> coeffs;
    for (int i = 0; i < splitter.count(); ++i)
      coeffs.push_back(Poco::NumberParser::parseFloat(splitter[i]));
    return coeffs;
  }

  /**
   *
   */
  void load_discharge_model(const string &filepath)
  {

  }

  /**
   *
   */
  void load_charge_model(const string &filepath)
  {

  }

  /**
   * If voltage reading is allowed, then this method takes a reading and inputs
   * it into the voltage filter. It subsequently takes the new output of the
   * voltage filter, attached the current time in seconds to it, and feeds it
   * into the voltage differentiator.
   */
  void read_voltage(Poco::Util::TimerTask &)
  {
    if (!reading_enabled)
      return;

    voltage_filter->input(get_voltage());
    voltage_diff.input(
        voltage_filter->output(),
        RasmTime::current_time_millis() / 1000.0
    );
  }

  /**
   * Updates the battery state field variables.
   */
  void estimate_state(Poco::Util::TimerTask &)
  {
    // get current voltage and voltage slope
    double voltage = voltage_filter->output();
    double slope = voltage_diff.derivative();

    // determine if charging or discharging
    if (slope <= 0)
      charging_ = false;
    else
      charging_ = true;

    // if charging, then apply a non-recursive particle filter to the charge model
    // otherwise, apply a non-recursive particle filter to the discharge model
    std::vector<BatteryDataRecord> &particles = charging_ ? charge_particles : discharge_particles;
    std::vector<double> errors(particles.size(), 0);

    // calculate the weighted error for each particle
    for (int i = 0; i < errors.size(); ++i)
    {
      BatteryDataRecord particle = particles[i];
      double voltage_diff = (voltage - particle.voltage) / particle.voltage;
      double slope_diff = (slope - particle.voltage_slope) / particle.voltage_slope;
      errors[i] = voltage_error_weight*voltage_diff + slope_error_weight*slope_diff;
    }

    // get the model time of the minimum error particle
    int min_err_time = particles[index_of_min(errors)].seconds;

    // calculate time remaining
    int model_end_time = particles[errors.size()-1].seconds;
    time_remaining_ = model_end_time - min_err_time;

    // calculate the battery percent
    if (charging_)
      battery_percent_ = 100 * time_remaining_ / (double)model_end_time;
    else
      battery_percent_ = 100 * (1 - time_remaining_ / (double)model_end_time);

    battery_voltage_ = voltage;
  }

  /**
   * Returns the smallest index of the smallest number contained in the given
   * vector of numbers. If the vector is empty, then -1 is returned.
   */
  int index_of_min(const std::vector<double> &list)
  {
    if (list.size() == 0)
      return -1;

    int min_number = list[0];
    int min_index = 0;
    for (int i = 1; i < list.size(); ++i)
    {
      if (list[i] < min_number)
      {
        min_number = list[i];
        min_index = i;
      }
    }
    return min_index;
  }

public:
  /**
   * Instantiates a new estimator and starts it's voltage reading and estimation
   * routines in a background thread.
   */
  BatteryEstimator(double (*voltage_getter)())
  : battery_voltage_(0)
  , battery_percent_(0)
  , time_remaining_(0)
  , charging_(false)
  {
    get_voltage = voltage_getter;

    // get the configuration group for the battery subsystem
    Poco::AutoPtr<MapConfiguration> batteryconfigs =
        ConfigurationManager::get_instance().
        get_config_group(ConfigurationManager::Group::BATTERY);

    // initialize the voltage filter with its feedforward/back coefficients
    string ff_coeffs_str = batteryconfigs->getRawString("voltage_feedforward_coeffs");
    std::vector<double> ff_coeffs = parse_number_list(ff_coeffs_str);
    string fb_coeffs_str = batteryconfigs->getRawString("voltage_feedback_coeffs");
    std::vector<double> fb_coeffs = parse_number_list(fb_coeffs_str);
    voltage_filter = new RealTimeLTIFilter(ff_coeffs, fb_coeffs);

    // read in some more configurations
    particle_trials = batteryconfigs->getDouble("particle_trials");
    voltage_error_weight = batteryconfigs->getDouble("voltage_error_weight");
    slope_error_weight = batteryconfigs->getDouble("slope_error_weight");
    dead_volts = batteryconfigs->getDouble("dead_volts");

    // load the battery models -- must be done after particle_trials is initialized
    string folder = batteryconfigs->getRawString("root_working_dir") + "/config";
    load_discharge_model("/discharge_model.csv");
    load_charge_model("/charge_model.csv");

    // schedule the voltage read task for periodic execution
    Poco::AutoPtr<Poco::Util::TimerTask> read_task =
        new Poco::Util::TimerTaskAdapter<BatteryEstimator>(
        *this, &BatteryEstimator::read_voltage);
    Poco::AutoPtr<Poco::Util::TimerTask> read_task_ptr(read_task);
    int reading_period = batteryconfigs->getInt("read_period_millis");
    timer.scheduleAtFixedRate(read_task_ptr, reading_period, reading_period);

    // schedule the battery state estimation task for periodic execution
    Poco::AutoPtr<Poco::Util::TimerTask> estimate_task =
        new Poco::Util::TimerTaskAdapter<BatteryEstimator>(
        *this, &BatteryEstimator::estimate_state);
    Poco::AutoPtr<Poco::Util::TimerTask> estimate_task_ptr(estimate_task);
    int estimation_period = batteryconfigs->getInt("estimation_period_millis");
    timer.scheduleAtFixedRate(estimate_task_ptr, estimation_period, estimation_period);
  }

  /**
   * Destructs this singleton by closing the timer.
   */
  ~BatteryEstimator()
  {
    timer.cancel();
    timer.~Timer();
  }

  /**
   * Makes it that this battery estimator won't take any voltage readings until
   * the enable_reading method is called. If this estimator is already disabled
   * in this way, then this method does nothing.
   */
  void disable_reading()
  {
    reading_enabled = false;
  }

  /**
   * Allows this estimator to take voltage readings.
   */
  void enable_reading()
  {
    reading_enabled = true;
  }

  /**
   * Returns the battery's present voltage in volts.
   */
  double battery_voltage()
  {
    return battery_voltage_;
  }

  /**
   * Returns the battery's present level of charge as a percent value. The
   * percentage ranges from 0%, for completely discharged, to 100% for completely
   * charged. The percentage is directly proportional to the amount of time that
   * the battery can be used before it's fully discharged.
   */
  double battery_percent()
  {
    return battery_percent_;
  }

  /**
   * Returns the time, in seconds, until the battery will be 'fully' discharged.
   * If the battery is charging at the moment, then the seconds returned is the
   * estimated length of time until the battery is fully charged.
   */
  int time_remaining()
  {
    return time_remaining_;
  }

  /**
   * Returns true if the battery is being charged; false if not.
   */
  bool charging()
  {
    return charging_;
  }
};


#endif
