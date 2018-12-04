/**
 * Defines the CausalLTIFilter class.
 */

#ifndef RASM2_UTIL_CAUSAL_LTI_FILTER_HPP
#define RASM2_UTIL_CAUSAL_LTI_FILTER_HPP

#include <vector>

#include "rasm2/util/circular_array.hpp"

using std::vector;

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

#endif
