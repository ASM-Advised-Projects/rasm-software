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
#include <fstream>

using std::string;
using std::vector;


/**
 * An instance of this class periodically reads a battery's voltage, estimates
 * it's level of charge, and provides basic queryable stats.
 */
class BatteryEstimator
{
private:
  /**
   * This struct holds a battery voltage data point containing a time, voltage,
   * and voltage slope.
   */
  struct BatteryDatum
  {
    unsigned int seconds;
    double volts;
    double voltsperhr;
  };

  double (*get_voltage)();
  bool reading_enabled;
  Poco::Util::Timer timer;

  RealTimeLTIFilter *voltage_filter;
  RealTimeDifferentiator voltage_diff;

  vector<BatteryDatum> charge_particles;
  vector<BatteryDatum> discharge_particles;
  double voltage_error_weight;
  double slope_error_weight;
  int dead_volts;

  double battery_voltage_;
  double battery_percent_;
  int time_remaining_;
  bool charging_;

  /**
   * Parses the comma-delimited numbers in the given string and places them
   * into the given vector in the same order as they appear in the string. The
   * given vector will be cleared beforehand.
   *
   * Throws a Poco::SyntaxException if a comma-delimited token other than a
   * parseable number is encountered in the string.
   */
  void parse_number_list(const string &num_list, vector<double> &vals)
  {
    Poco::StringTokenizer splitter(num_list, ",");
    vals.clear();
    vals.reserve(splitter.count());
    for (int i = 0; i < splitter.count(); ++i)
      vals.push_back(Poco::NumberParser::parseFloat(splitter[i]));
  }

  /**
   * Reads battery data from the given text file into the given rows vector. The
   * vector will be cleared beforehand.
   *
   * The text file should be in csv format with three data columns and no
   * headers. The first column should be an integer time in seconds, the second
   * column should be the floating-point voltage in volts, and the third should
   * be the floating-point voltage derivative with respect to time. The
   * derivative should have units of volts/hour.
   *
   * Throws a Poco::DataException if the file data is not formatted properly.
   * Throws a Poco::FileException if the given file path doesn't represent a
   * readable text file.
   */
  void load_battery_data(const string &filepath, vector<BatteryDatum> &rows)
  {
    rows.clear();

    // open the data file
    std::ifstream file(filepath);
    if (!file.is_open())
    {
      throw Poco::FileException("In BatteryEstimator::load_file_data\n"
      "File at path '" + filepath + "' can't be opened for reading.");
    }

    // parse each line of file and add to the rows vector
    string line;
    vector<double> line_vals;
    while (std::getline(file, line))
    {
      parse_number_list(line, line_vals);
      if (line_vals.size() != 3)
      {
        throw Poco::DataException("In BatteryEstimator::load_file_data\n"
        "The file '" + filepath + "' has at least one data row that doesn't"
        "hold exactly three values.");
      }

      BatteryDatum data_point = {
        (unsigned int)line_vals[0],
        (double)line_vals[1],
        (double)line_vals[2]
      };
      rows.push_back(data_point);
    }

    file.close();
  }

  /**
   * Reads in battery discharge data from a text file at the given filepath. The
   * data is interpolated to get a series of <num_particles> particles that will
   * be stored in the given particles vector. The minimum amount of data rows
   * that the file must have is two.
   *
   * Throws a Poco::FileException if the given file path doesn't represent a
   * readable text file.
   * Throws a Poco::SyntaxException if a comma-delimited token other than a
   * parseable number is encountered in the string.
   * Throws a Poco::DataException if the file data has less than two rows or if
   * the data is not formatted properly.
   */
  void load_battery_model(const string &filepath,
      vector<BatteryDatum> &particles, int num_particles)
  {
    // load the battery data
    vector<BatteryDatum> data;
    load_battery_data(filepath, data);
    if (data.size() < 2)
    {
      throw Poco::DataException("In BatteryMonitor::load_battery_model\n"
      "The file '" + filepath + "' holds less than two data points.");
    }

    particles.reserve(num_particles);

    // get the start and end times
    unsigned int start_time = data[0].seconds;
    unsigned int end_time = data[data.size()-1].seconds;

    // find the time difference between adjacent particles
    double time_diff = (end_time - start_time) / (double)(num_particles-1);

    // set the time values of all particles
    particles[0].seconds = start_time;
    for (int i = 1; i < particles.size(); i++)
      particles[i].seconds = (unsigned int)(start_time + i*time_diff);

    // for each data point besides the first, interpolate with the previous
    // data point in order to find the voltage and voltage slope for all of the
    // particles that lie between them
    int part_ind = 0;
    for (int data_ind = 1; data_ind < data.size(); data_ind++)
    {
      for (; particles[part_ind].seconds <= data[data_ind].seconds; part_ind++)
      {
        // where between the two data points the particle lies
        double time_fraction = (particles[part_ind].seconds - data[data_ind-1].seconds) / time_diff;

        // compute particle voltage via interpolation
        // V = V_{i-1} + (V_i - V_{i-1}) * time_fraction
        particles[part_ind].volts = data[data_ind-1].volts +
            (data[data_ind].volts - data[data_ind-1].volts) * time_fraction;

        // compute particle voltage slope via interpolation
        // VS = VS_{i-1} + (VS_i - VS_{i-1}) * time_fraction
        particles[part_ind].voltsperhr = data[data_ind-1].voltsperhr +
            (data[data_ind].voltsperhr - data[data_ind-1].voltsperhr) * time_fraction;
      }
    }
  }

  /**
   * If voltage reading is allowed, then this method takes a reading and inputs
   * it into the voltage filter. It subsequently takes the new output of the
   * voltage filter, attaches the current time in hours to it, and feeds it
   * into the voltage differentiator.
   */
  void read_voltage(Poco::Util::TimerTask &)
  {
    if (!reading_enabled)
      return;

    voltage_filter->input(get_voltage());
    voltage_diff.input(
        voltage_filter->output(),
        RasmTime::current_time_millis() / 1000.0 / 3600.0
    );
  }

  /**
   * Updates the battery state field variables using a rudimentary particle
   * filter.
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
    vector<BatteryDatum> &particles = charging_ ? charge_particles : discharge_particles;
    vector<double> errors(particles.size(), 0);

    // calculate the weighted error for each particle
    for (int i = 0; i < errors.size(); ++i)
    {
      BatteryDatum particle = particles[i];
      double voltage_diff = (voltage - particle.volts) / particle.volts;
      double slope_diff = (slope - particle.voltsperhr) / particle.voltsperhr;
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
  int index_of_min(const vector<double> &list)
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
    string fb_coeffs_str = batteryconfigs->getRawString("voltage_feedback_coeffs");
    vector<double> ff_coeffs;
    vector<double> fb_coeffs;
    parse_number_list(ff_coeffs_str, ff_coeffs);
    parse_number_list(fb_coeffs_str, fb_coeffs);
    voltage_filter = new RealTimeLTIFilter(ff_coeffs, fb_coeffs);

    // load the battery models
    int num_particles = batteryconfigs->getDouble("num_particles");
    string folder = batteryconfigs->getRawString("root_working_dir") + "/config";
    load_battery_model("/discharge_model.csv", discharge_particles, num_particles);
    load_battery_model("/charge_model.csv", charge_particles, num_particles);

    // read in some more configurations
    voltage_error_weight = batteryconfigs->getDouble("voltage_error_weight");
    slope_error_weight = batteryconfigs->getDouble("slope_error_weight");
    dead_volts = batteryconfigs->getDouble("dead_volts");

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
