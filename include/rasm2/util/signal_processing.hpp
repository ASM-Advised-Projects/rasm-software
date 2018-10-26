/**
 * Defines the CausalLTIFilter, RealTimeDifferentiator, and RealTimeIntegrator
 * classes.
 */

#ifndef RASM2_UTIL_SIGNAL_PROCESSING_HPP
#define RASM2_UTIL_SIGNAL_PROCESSING_HPP

#include "circular_array.hpp"

/**
 * This class represents a causal SISO LTI digital filter. SISO means
 * single-input single-output while LTI means linear time-invariant.
 *
 * The filtering algorithm uses the given feedforward and feedback
 * coefficients (ff_coeffs and fb_coeffs) to compute the output signal y using
 * signal input x according to this equation:
 *   y[n] = sum{i=0:M}(ff_coeffs[i] * x[n-i]) + sum{j=1:N}(fb_coeffs[i] * y[n-j])
 * Where M is the number of feedforward coeffs and N is the number of feedback
 * coeffs. As the names suggest, feedforward coefficients relate input values
 * directly to the current output value while feedback coefficients relate
 * previous output values to the current output value.
 */
class CausalLTIFilter
{
private:
  vector<double> ff_coeffs;
  vector<double> fb_coeffs;

  CircularArray<double> inputs;
  CircularArray<double> outputs;

public:
  CausalLTIFilter(const CausalLTIFilter &) = default;
  CausalLTIFilter & operator=(const CausalLTIFilter &) = default;

  /**
   * Creates a new filter using the given feedforward (ff_coeffs) and feedback
   * (fb_coeffs) coefficients.
   */
  CausalLTIFilter(const vector<double> &ff_coeffs, const vector<double> &fb_coeffs)
  : ff_coeffs(ff_coeffs)
  , fb_coeffs(fb_coeffs)
  , inputs(ff_coeffs.size())
  , outputs(1+fb_coeffs.size())
  {
  }

  /**
   * Adds the given value as the newest input value of this filter. To get the
   * new output of this filter, call output().
   */
  void input(double value)
  {
    inputs.push(value);
    double new_output = 0;
    for (int i = 0; i < ff_coeffs.size(); ++i)
      new_output += ff_coeffs[i] * inputs[i];
    for (int j = 0; j < fb_coeffs.size(); ++j)
      new_output += fb_coeffs[j] * outputs[j];
    outputs.push(new_output);
  }

  /**
   * Returns the current output value of this filter.
   */
  double output() const
  {
    return outputs[0];
  }

  /**
   * This operator calls input(value), then returns output().
   */
  double operator()(double value)
  {
    input(value);
    return output();
  }
};


/**
 * Computes the derivative of a series of dependent input values with respect
 * to a series of independent values (e.g., time). A second order lagrange
 * interpolating polynomial is used to find the current derivative. This
 * requires three data points. If there are only two points then a finite-
 * difference formula is used. If less than two points have been input, then the
 * derivative is zero.
 */
class RealTimeDifferentiator
{
private:
  CircularArray<double> x;
  CircularArray<double> f;
  double derivative_;

public:
  RealTimeDifferentiator(const RealTimeDifferentiator &) = default;
  RealTimeDifferentiator & operator=(const RealTimeDifferentiator &) = default;

  /**
   * Creates a new differentiator with no initial input data.
   */
  RealTimeDifferentiator()
  : x(3)
  , f(3)
  , derivative_(0)
  {
  }

  /**
   * Inputs the given dependent-independent pair of values as the newest data
   * point. If the independent value is the same as the last independent value
   * then a std::invalid_argument exception will the thrown.
   */
  void input(double ind_value, double dep_value)
  {
    if (f.size() > 0 && ind_value == x[0])
      throw std::invalid_argument("Adjacent independent values must differ.");

    x.push(ind_value);
    f.push(dep_value);

    if (f.size() > 2)
    {
      double x0 = x[0];  // latest value
      double x1 = x[1];
      double x2 = x[2];
      derivative_ =
          f[2]*(2*x0-x1-x0) / ((x2-x1)*(x2-x0)) +
          f[1]*(2*x0-x2-x0) / ((x1-x2)*(x1-x0)) +
          f[0]*(2*x0-x2-x1) / ((x0-x2)*(x0-x1));
    }
    else if (f.size() > 1)
    {
      derivative_ = (f[0]-f[1]) / (x[0]-x[1]);
    }
    else
    {
      derivative_ = 0;
    }
  }

  /**
   * Returns the current derivative of the input data if more than one value
   * has been input; returns 0 otherwise.
   */
  double derivative() const
  {
    return derivative_;
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
class RealTimeIntegrator
{
private:
  CircularArray<double> x;  // independent variable
  CircularArray<double> f;  // dependent variable
  bool relative;
  double temp_integral;  // the absolute integral value
  double final_integral;  // a relative or absolute integral value

public:
  RealTimeIntegrator(const RealTimeIntegrator &) = default;
  RealTimeIntegrator & operator=(const RealTimeIntegrator &) = default;

  /**
   * Constructs a new integrator that integrates over the <segments> latest
   * intervals between data points. If relative is true, then the dependent data
   * values will be viewed as relative to the data value at the start of the
   * current integration range.
   */
  RealTimeIntegrator(unsigned int segments, bool relative)
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
};

#endif  // UTIL_SIGNAL_PROCESSING_HPP
