/**
 * Defines the RealTimeIntegrator and RealTimeRangeIntegrator classes.
 */

#ifndef RASM2_UTIL_INTEGRATORS_HPP
#define RASM2_UTIL_INTEGRATORS_HPP

#include <stdexcept>

#include "rasm2/util/circular_array.hpp"

/**
 * Computes the integral of a series of dependent input values with respect to
 * a series of independent values (e.g., time). The integral is taken over all
 * data segments since construction or the last reset.
 *
 * The trapezoidal rule is used to calculate the integral over a series of
 * values for a given range. Simplson's rule is not used here because it takes
 * a complicated modification in order to deal with non-uniformly spaced data.
 */
class RealTimeIntegrator
{
private:
  double prev_ind_value;
  double prev_dep_value;
  double _integral;
  bool prev_input;

public:
  RealTimeIntegrator(const RealTimeIntegrator &) = default;
  RealTimeIntegrator & operator=(const RealTimeIntegrator &) = default;

  /**
   * Constructs a new infinite-segment integrator.
   */
  RealTimeIntegrator()
  : _integral(0)
  , prev_input(false)
  {
  }

  /**
   * Inputs the given dependent-independent pair of values as the newest data
   * point. If the independent value is the same as the last independent value
   * then a std::invalid_argument exception will the thrown.
   */
  void input(double ind_value, double dep_value)
  {
    if (!prev_input)
    {
      prev_ind_value = ind_value;
      prev_dep_value = dep_value;
      prev_input = true;
      return;
    }

    if (ind_value == prev_ind_value)
      throw std::invalid_argument("Adjacent independent values must differ.");

    // add latest trapezoid to the absolute integral
    _integral += (ind_value-prev_ind_value) * (prev_dep_value+dep_value) / 2;

    prev_ind_value = ind_value;
    prev_dep_value = dep_value;
  }

  /**
   * Returns the current integral of the input data if more than one value
   * has been input; returns 0 otherwise.
   */
  double integral() const
  {
    return _integral;
  }

  void reset()
  {
    _integral = 0;
    prev_input = false;
  }
};

/**
 * Computes the integral of a series of dependent input values with respect to
 * a series of independent values (e.g., time). The integral is taken over a
 * specified number of data segments given by the <segments> constructor
 * argument. If the values being integrated need to be considered as relative
 * to the first value in the current range then the <relative> constructor
 * argument should be set true.
 *
 * These are the equations that define the calculated integral I given a series
 * of independent values, x, and dependent values, f. Note that i=0 is the
 * index of the latest data point.
 *   If relative is false:
 *     I = sum{i=0:(segments-1)}( (x[i]-x[i+1]) * 0.5*(f[i]+f[i+1]) )
 *   If relative is true:
 *     I = sum{i=0:(segments-1)}( (x[i]-x[i+1]) * 0.5*(f[i]+f[i+1])-f[segments-1] )
 *
 * For example, if segments=2 (3 data points used), the following absolute
 * integral values (3rd row) and relative integral values (4th row) will result
 * from the two rows of data:
 *   Independent:  0    1    2    3    4    5
 *   Dependent:    2    3    4    4    3    2
 *   Abs Integral: 0.0  2.5  6.0  7.5  7.5  6.0
 *   Rel Integral: 0.0  0.5  2.0  1.5 -0.5 -2.0
 *
 * The trapezoidal rule is used to calculate the integral over a series of
 * values for a given range. Simplson's rule is not used here because it takes
 * a complicated modification in order to deal with non-uniformly spaced data.
 */
class RealTimeRangeIntegrator
{
private:
  CircularArray<double> x;  // independent variable
  CircularArray<double> f;  // dependent variable
  bool relative;
  double temp_integral;  // the absolute integral value
  double final_integral;  // a relative or absolute integral value

public:
  RealTimeRangeIntegrator(const RealTimeRangeIntegrator &) = default;
  RealTimeRangeIntegrator & operator=(const RealTimeRangeIntegrator &) = default;

  /**
   * Constructs a new integrator that integrates over the <segments> latest
   * intervals between data points. If relative is true, then the dependent data
   * values will be viewed as relative to the data value at the start of the
   * current integration range.
   */
  RealTimeRangeIntegrator(unsigned int segments, bool relative)
  : x(segments+1)
  , f(segments+1)
  , relative(relative)
  , temp_integral(0)
  , final_integral(0)
  {
    if (segments == 0)
      throw std::invalid_argument("The number of segments must be greater than 0.");
  }

  /**
   * Inputs the given dependent-independent pair of values as the newest data
   * point. If the independent value is the same as the last independent value
   * then a std::invalid_argument exception will the thrown.
   */
  void input(double ind_value, double dep_value)
  {
    if (f.size() == 0)
    {
      x.push(ind_value);
      f.push(dep_value);
      return;
    }

    if (ind_value == x[0])
      throw std::invalid_argument("Adjacent independent values must differ.");

    // using this variable so that the actual integral value can be changed
    // all at once at the end of the method (makes this class thread safe)
    double integral_change = 0;

    // subtract the earliest trapezoid from the absolute integral before
    // removing the last data point (last data point will be removed when the
    // new data point is added if x & f are at capacity).
    if (x.size() == x.capacity())
    {
      unsigned int a = x.size()-2;
      unsigned int b = x.size()-1;
      integral_change -= (x[a]-x[b]) * (f[b]+f[a]) / 2;
    }

    // add new data point
    x.push(ind_value);
    f.push(dep_value);

    // add latest trapezoid to the absolute integral
    integral_change += (x[0]-x[1]) * (f[1]+f[0]) / 2;
    temp_integral += integral_change;

    // set the final integral to either the relative or absolute integral
    if (relative)
      final_integral = temp_integral - (x[0]-x[x.size()-1])*f[x.size()-1];
    else
      final_integral = temp_integral;
  }

  /**
   * Returns the current integral of the input data if more than one value
   * has been input; returns 0 otherwise.
   */
  double integral() const
  {
    return final_integral;
  }

  void reset()
  {
    x.clear();
    f.clear();
    temp_integral = 0;
    final_integral = 0;
  }
};

#endif
