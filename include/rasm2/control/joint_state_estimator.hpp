/**
 * Defines the JointStateEstimator class.
 */

#ifndef RASM2_CONTROL_JOINT_STATE_ESTIMATOR_HPP
#define RASM2_CONTROL_JOINT_STATE_ESTIMATOR_HPP

#include <array>
#include <functional>

#include "rasm2/control/joint.hpp"
#include "rasm2/util/causal_lti_filter.hpp"
#include "rasm2/util/differentiators.hpp"

/**
 * This class estimates the position, velocity, and acceleration of a joint from
 * its encoder output.
 */
class JointStateEstimator
{
private:
  std::function<double (int)> enc2pos;
  std::unique_ptr<CausalLTIFilter> position_filter;
  RealTimeDifferentiator position_diff;
  RealTimeDifferentiator velocity_diff;
  int required_inputs;
  bool empty;

public:
  /**
   * Constructs a new joint state estimator that uses the given enc2pos function
   * to transform an encoder output into a joint position.
   */
  JointStateEstimator(std::function<double (int)> enc2pos)
  : enc2pos(enc2pos)
  {
    std::vector<double> ff_coeffs;
    ff_coeffs.push_back(0.25);
    ff_coeffs.push_back(0.125);
    ff_coeffs.push_back(0.125);

    std::vector<double> fb_coeffs;
    fb_coeffs.push_back(0.25);
    fb_coeffs.push_back(0.25);

    position_filter.reset(new CausalLTIFilter(ff_coeffs, fb_coeffs));

    if (ff_coeffs.size() > fb_coeffs.size())
      required_inputs = ff_coeffs.size();
    else
      required_inputs = fb_coeffs.size();

    empty = false;
  }

  /**
   * Updates the estimations using a new encoder output corresponding to
   * the given time.
   */
  void update(int encoder_output, double time)
  {
    // input joint positions into the filters
    position_filter->input(enc2pos(encoder_output));

    // if this is the first update then completely fill the filter with repeat
    // values of the present encoder output so that the joint position is
    // immediately accurate
    if (empty)
    {
      double pos = enc2pos(encoder_output);
      for (int i = 1; i < required_inputs; ++i)
        position_filter->input(pos);
      empty = !empty;
    }

    // input the filtered joint position into the first differentiator
    position_diff.input(time, position_filter->output());

    // input the filtered joint velocity into the second differentiator
    velocity_diff.input(time, position_diff.derivative());
  }

  /**
   *
   */
  double get_position() const
  {
    return position_filter->output();
  }

  /**
   *
   */
  double get_velocity() const
  {
    return position_diff.derivative();
  }

  /**
   *
   */
  double get_acceleration() const
  {
    return velocity_diff.derivative();
  }
};

#endif
