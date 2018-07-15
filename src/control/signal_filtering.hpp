/**
 * Defines the RealTimeLTIFilter and CircularBuffer classes.
 */

#ifndef _SIGNAL_FILTERING_H_
#define _SIGNAL_FILTERING_H_

#include <mutex>
#include <vector>

using std::vector;


/**
 *
 */
template <typename NumericType>
class CircularArray {
private:
  static_assert(std::is_arithmetic<NumericType>::value, "NumericType must be numeric");
	std::unique_ptr<double[]> array;
	unsigned int next_index;
	unsigned int max_size_;

public:
  /**
    *
   */
	CircularArray(unsigned int max_size)
	: array(std::unique_ptr<NumericType[]>(new NumericType[max_size]))
  , next_index(0)
	, max_size_(max_size)
	{
    for (int i = 0; i < max_size; i++)
      array[i] = 0;
  }

  /**
   *
   */
	void push(NumericType val)
	{
	  array[next_index] = val;
    next_index = (next_index + 1) % max_size_;
	}

  /**
   *
   */
  NumericType operator[](unsigned int entries_back)
  {
    if (entries_back >= max_size_)
      return 0;
    return array[(next_index-1-entries_back) % max_size_];
  }

  unsigned int max_size()
  {
    return max_size();
  }
};

/**
 * Real-time means two things: this filter is causal and this class's interface
 * only supports input/output of one signal value at a time.
 *
 * General filter algorithm
 *     y[n] = sum{i=0:M}(ff_coeffs[i] * x[n-i]) + sum{j=1:N}(fb_coeffs[i] * y[n-j])
 */
class RealTimeLTIFilter
{
private:
  vector<double> ff_coeffs;
  vector<double> fb_coeffs;

  CircularArray<double> inputs;
  CircularArray<double> outputs;

  int max_inputs;
  int max_outputs;

  int max(int a, int b)
  {
    return a > b ? a : b;
  }

public:
  /**
   *
   */
  RealTimeLTIFilter(const vector<double> &ff_coeffs, const vector<double> &fb_coeffs, int store_length)
  : ff_coeffs(ff_coeffs)
  , fb_coeffs(fb_coeffs)
  , inputs(ff_coeffs.size())
  , outputs(max(fb_coeffs.size(), store_length))
  {
    max_inputs = inputs.max_size();
    max_outputs = outputs.max_size();
  }

  /**
   *
   */
  void input(double value)
  {
    inputs.push(value);

    double new_output = 0;

    for (int i = 0; i < max_inputs; i++)
      new_output += ff_coeffs[i] * inputs[i];

    // j still starts at 0 because new_output hasn't been pushed to outputs yet
    for (int j = 0; j < max_outputs; j++)
      new_output += fb_coeffs[j] * outputs[j];

    outputs.push(new_output);
  }

  /**
   *
   */
  double output(unsigned int values_back)
  {
    return outputs[values_back];
  }

  /**
   *
   */
  double operator()(double value)
  {
    input(value);
    return output(0);
  }
};

#endif
