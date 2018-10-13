/**
 * Defines the following classes related to signal processing:
 *   CircularArray<NumericType>
 *   RealTimeLTIFilter
 *   RealTimeDifferentiator
 *   RealTimeIntegrator
 */

#ifndef SIGNAL_PROCESSING_HPP
#define SIGNAL_PROCESSING_HPP

#include <iostream>
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
  , size_(0)
  {
  }

  /**
   * Copy constructor.
   */
  CircularArray(const CircularArray &other)
  {
    next_index = other.next_index;
    max_size_ = other.max_size_;
    size_ = other.size_;

    array = new NumericType[other.max_size_];
    for (int i = 0; i < size_; i++)
      array[i] = other.array[i];
  }

  /**
   * Destructs this circular array.
   */
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
    if (size_ < max_size_)
      ++size_;
  }

  /**
   * Returns the value at the given index, which is a number of entries back.
   * If the entries back is greater than size()-1 then 0 is returned.
   */
  NumericType get(unsigned int entries_back)
  {
    if (entries_back >= size_)
      return 0;
    return array[(next_index-1-entries_back) % max_size_];
  }

  /**
   * The equivalent of calling the get(unsigned int) method.
   */
  NumericType operator[](unsigned int entries_back)
  {
    return get(entries_back);
  }

  /**
   * Changes the value held at the given index (entries_back) to val. If the
   * given index is greater than size()-1 then this method does nothing.
   */
  void modify(unsigned int entries_back, NumericType val)
  {
    if (entries_back < size_)
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

  /**
   * Equals assignment operator.
   */
  CircularArray & operator=(const CircularArray &other)
  {
    if (*this == other)
      return *this;

    next_index = other.next_index;
    max_size_ = other.max_size_;
    size_ = other.size_;

    array = new NumericType[other.max_size_];
    for (int i = 0; i < size_; i++)
      array[i] = other.array[i];

    return *this;
  }

  /**
   * Equals comparison operator.
   */
  bool operator==(const CircularArray &other)
  {
    for (int i = 0; i < size_; i++)
    {
      if (array[i] != other.array[i])
        return false;
    }

    return next_index == other.next_index &&
        max_size_ == other.max_size_ &&
        size_ == other.size_;
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
  RealTimeLTIFilter(const RealTimeLTIFilter &) = default;
  RealTimeLTIFilter & operator=(const RealTimeLTIFilter &) = default;

  /**
   * Creates a new filter using the given feedforward (ff_coeffs) and feedback
   * (fb_coeffs) coefficients.
   */
  RealTimeLTIFilter(const vector<double> &ff_coeffs, const vector<double> &fb_coeffs)
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
    std::cout << "input1" << std::endl;
    double new_output = 0;
    std::cout << "input2" << std::endl;
    for (int i = 0; i < ff_coeffs.capacity(); ++i)
      new_output += ff_coeffs[i] * inputs[i];
    std::cout << "input3" << std::endl;
    for (int j = 0; j < fb_coeffs.capacity(); ++j)
      new_output += fb_coeffs[j] * outputs[j];

    std::cout << "input4" << std::endl;
    outputs.push(new_output);
    std::cout << "input5" << std::endl;
  }

  /**
   * Returns the current output value of this filter.
   */
  double output()
  {
    return outputs[0];
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
  CircularArray<double> f;
  CircularArray<double> x;
  double derivative_;

public:
  RealTimeDifferentiator(const RealTimeDifferentiator &) = default;
  RealTimeDifferentiator & operator=(const RealTimeDifferentiator &) = default;

  /**
   * Creates a new differentiator with no initial input data.
   */
  RealTimeDifferentiator()
  : f(3)
  , x(3)
  , derivative_(0)
  {
  }

  /**
   * Inputs the given dependent-independent pair of values as the newest data
   * point.
   */
  void input(double dep_value, double ind_value)
  {
    f.push(dep_value);
    x.push(ind_value);

    if (f.size() > 2)
    {
      double x0 = x[0];  // latest value
      double x1 = x[1];
      double x2 = x[2];
      derivative_ =
          f[2]*(2*x0-x1-x0) / ((x2-x1)*(x2-x0)) +
          f[1]*(2*x0-x2-x0) / ((x1-x2)*(x1-x0)) +
          f[0]*(2*x0-x2-x1) / ((x0-x2)*(x0-x1));
      return;
    }

    if (f.size() > 1)
    {
      derivative_ = (f[0]-f[1]) / (x[0]-x[1]);
      return;
    }

    derivative_ = 0;
  }

  /**
   * Returns the current derivative of the input data if more than one value
   * has been input; returns 0 otherwise.
   */
  double derivative()
  {
    return derivative_;
  }
};


/**
 * Computes the integral of a series of dependent input values with respect
 * to a series of independent values (e.g., time). The trapezoidal rule is used
 * to calculate the integral over a series of values for a given range.
 * Simplson's rule is not used here because it takes a complicated modification
 * in order to deal with non-uniformly spaced data.
 */
class RealTimeIntegrator
{
private:
  CircularArray<double> f;
  CircularArray<double> x;
  double integral_;

public:
  RealTimeIntegrator(const RealTimeIntegrator &) = default;
  RealTimeIntegrator & operator=(const RealTimeIntegrator &) = default;

  /**
   * Constructs a new integrator that integrates over the <range> latest data
   * points.
   */
  RealTimeIntegrator(unsigned int range)
  : f(range)
  , x(range)
  {
    integral_ = 0;
  }

  /**
   * Inputs the given dependent-independent pair of values as the newest data
   * point.
   */
  void input(double dep_value, double ind_value)
  {
    // using this variable so that the actual integral value can be changed
    // all at once at the end of the method (makes this class thread safe)
    double integral_change = 0;

    if (f.size() == f.capacity())
    {
      // subtract earliest trapezoid before removing last data point
      unsigned int a = f.size()-2;
      unsigned int b = f.size()-1;
      integral_change -= (b-a) * (f[a]+f[b]) / 2;
    }
    f.push(dep_value);
    x.push(ind_value);

    // add latest trapezoid to integral
    integral_change += (x[0]-x[1]) * (f[1]+f[0]) / 2;

    integral_ += integral_change;
  }

  /**
   * Returns the current integral of the input data if more than one value
   * has been input; returns 0 otherwise.
   */
  double integral()
  {
    return integral_;
  }
};

#endif  // SIGNAL_PROCESSING_HPP
