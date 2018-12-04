/**
 * Defines the RealTimeDifferentiator class.
 */

#ifndef RASM2_UTIL_DIFFERENTIATORS_HPP
#define RASM2_UTIL_DIFFERENTIATORS_HPP

#include <stdexcept>

#include "rasm2/util/circular_array.hpp"

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

  void reset()
  {
    x.clear();
    f.clear();
    derivative_ = 0;
  }
};

#endif
