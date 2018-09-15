/**
 * Defines the following classes related to signal processing:
 *   CircularArray<NumericType>
 *   RealTimeLTIFilter
 *   RealTimeDifferentiator
 *   RealTimeIntegrator
 */

#ifndef _SIGNAL_PROCESSING_H_
#define _SIGNAL_PROCESSING_H_

#include <vector>

using std::vector;


/**
 * This class represents a ring buffer of numeric values. It's basically a
 * fixed-size array that will remove the oldest values in order to add new
 * values. Its values are indexed according to how recently they were added.
 */
template <typename NumericType>
class CircularArray {
private:
  static_assert(std::is_arithmetic<NumericType>::value, "NumericType must be numeric");
  NumericType *array;
	unsigned int next_index;
	unsigned int max_size_;
  unsigned int size_;

public:
  /**
   * Constructs an empty circular array with the given fixed size as its
   * capacity.
   */
	CircularArray(unsigned int max_size)
	: array(new NumericType[max_size])
  , next_index(0)
	, max_size_(max_size)
	{
    size_ = 0;
  }

  ~CircularArray()
  {
    delete[] array;
  }

  /**
   * Pushes a new value onto the front of this array. This will delete the
   * oldest value if this array is full.
   */
	void push(NumericType val)
	{
	  array[next_index] = val;
    next_index = (next_index + 1) % max_size_;
    ++size_;
	}

  /**
   * Returns the value at the given index, which is a number of entries back.
   * If the entries back is greater than size()-1 then 0 is returned.
   */
  NumericType get(unsigned int entries_back)
  {
    if (entries_back >= max_size_)
      return 0;
    return array[(next_index-1-entries_back) % max_size_];
  }

  /**
   * Changes the value held at the given index (entries_back) to val. If the
   * given index is greater than size()-1 then this method does nothing.
   */
  void modify(unsigned int entries_back, NumericType val)
  {
    if (entries_back < max_size_)
      array[(next_index-1-entries_back) % max_size_] = val;
  }

  /**
   * Returns the current amount of values in this array.
   */
  unsigned int size() const
  {
    return size_;
  }

  /**
   * Returns that max amount of values this array can hold.
   */
  unsigned int capacity() const
  {
    return max_size_;
  }
};


/**
 * This class represents a causal SISO LTI digital filter. SISO means
 * single-input single-output while LTI means linear time-invariant.
 * The 'RealTime' in the class name is just another word for causal.
 *
 * The filtering algorithm uses the given feedforward and feedback
 * coefficients (ff_coeffs and fb_coeffs) to compute the output signal y using
 * signal input x according to this equation:
 * y[n] = sum{i=0:M}(ff_coeffs[i] * x[n-i]) + sum{j=1:N}(fb_coeffs[i] * y[n-j])
 *
 * As the names suggest, feedforward coefficients relate input values directly
 * to the current output value while feedback coefficients relate previous
 * output values to the current output value.
 */
class RealTimeLTIFilter
{
private:
  vector<double> ff_coeffs;
  vector<double> fb_coeffs;

  CircularArray<double> inputs;
  CircularArray<double> outputs;

public:
  /**
   * Creates a new filter using the given feedforward (ff_coeffs) and feedback
   * (fb_coeffs) coefficients.
   */
  RealTimeLTIFilter(const vector<double> &ff_coeffs, const vector<double> &fb_coeffs)
  : ff_coeffs(ff_coeffs)
  , fb_coeffs(fb_coeffs)
  , inputs(ff_coeffs.size())
  , outputs(fb_coeffs.size())
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
    for (int i = 0; i < inputs.capacity(); i++)
      new_output += ff_coeffs[i] * inputs.get(i);
    for (int j = 0; j < outputs.capacity(); j++)
      new_output += fb_coeffs[j] * outputs.get(j);

    outputs.push(new_output);
  }

  /**
   * Returns the current output value of this filter.
   */
  double output()
  {
    return outputs.get(0);
  }

  /**
   * This operator is calls input(value), then returns output().
   */
  double operator()(double value)
  {
    input(value);
    return output();
  }
};


/**
 * infinite or range integration
 * first order differentiation
 * constant or non-constant time intervals for integration and differentiation
 *
 */
class RealTimeDifferentiator
{
private:


public:

};


/**
 *
 */
class RealTimeIntegrator
{
private:

public:

};

#endif
