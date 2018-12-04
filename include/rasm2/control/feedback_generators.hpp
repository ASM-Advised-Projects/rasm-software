/**
 * Defines the following classes:
 *   PIDFeedbackGenerator
 *   PowIDFeedbackGenerator
 *   PredictivePIDFeedbackGenerator
 *   FullStateFeedbackGenerator
 */

#ifndef RASM2_CONTROL_FEEDBACK_GENERATORS_HPP
#define RASM2_CONTROL_FEEDBACK_GENERATORS_HPP

#include <array>
#include <cmath>
#include <complex>
#include <memory>

#include "rasm2/control/joint.hpp"
#include "rasm2/control/trajectory_containers.hpp"
#include "rasm2/util/differentiators.hpp"
#include "rasm2/util/integrators.hpp"

/**
 * This class implements PID feedback control for a one-dimensional trajectory.
 * The PID control uses the error between the current position and the desired
 * position to generate the feedback output for tracking the trajectory.
 */
class PIDFeedbackGenerator
{
private:
  const Trajectory1D *pos_traj;
  RealTimeDifferentiator differentiator;
  RealTimeIntegrator integrator;
  double P, I, D;

public:
  /**
   * Constructs a new PID feedback generator with the given weight magnitudes.
   */
  PIDFeedbackGenerator(double prop_weight, double integ_weight, double deriv_weight)
  : pos_traj(nullptr)
  , P(prop_weight)
  , I(integ_weight)
  , D(deriv_weight)
  {
  }

  /**
   * Resets this generator and gives it a new desired trajectory.
   */
  void set_trajectory(const Trajectory1D &pos)
  {
    this->pos_traj = &pos;
    differentiator.reset();
    integrator.reset();
  }

  /**
   * Sets output to the feedback control value generated for the actual position
   * and currently set trajectory. The given time should be the elapsed time
   * since the start of the trajectory since it is used to interpret where the
   * joints should actually be.
   */
  double generate(double time, double position)
  {
    // find current error (actual - desired)
    double error = position - pos_traj->value(time);

    // update error integral
    integrator.input(time, error);

    // update error derivative
    differentiator.input(time, error);

    // compute weighted PID sum
    return P*error + I*integrator.integral() + D*differentiator.derivative();
  }
};


/**
 * This class implements power-integral-derivative feedback control for a
 * one-dimensional trajectory. It is the same as normal PID control except the
 * proportional term is now P*err^order where order can be any real number.
 */
class PowIDFeedbackGenerator
{
private:
  const Trajectory1D *pos_traj;
  RealTimeDifferentiator differentiator;
  RealTimeIntegrator integrator;
  double order;
  double P, I, D;

public:
  /**
   * Constructs a new PID feedback generator with the given power order and
   * weight magnitudes.
   */
  PowIDFeedbackGenerator(double pow_order, double pow_weight, double integ_weight, double deriv_weight)
  : pos_traj(nullptr)
  , order(pow_order)
  , P(pow_weight)
  , I(integ_weight)
  , D(deriv_weight)
  {
  }

  /**
   * Resets this generator and gives it a new desired trajectory.
   */
  void set_trajectory(const Trajectory1D &pos)
  {
    pos_traj = &pos;
    differentiator.reset();
    integrator.reset();
  }

  /**
   * Sets output to the feedback control value generated for the actual position
   * and currently set trajectory. The given time should be the elapsed time
   * since the start of the trajectory since it is used to interpret where the
   * joints should actually be.
   */
  double generate(double time, double position)
  {
    // find current error (actual - desired)
    double error = position - pos_traj->value(time);

    // update error integral
    integrator.input(time, error);

    // update error derivative
    differentiator.input(time, error);

    // compute weighted power-integral-derivative sum
    return P*pow(error, order) + I*integrator.integral() + D*differentiator.derivative();
  }
};


/**
 * This class implements a type of modified PID feedback control for a one-
 * dimensional trajectory. The PID control uses the error between the current
 * position and a modified desired position to generate the feedback output for
 * tracking the trajectory.
 *
 * The desired position is modified by adding two terms to the original position:
 *   vel_factor * (desired velocity [deg/sec])
 *   acc_factor * (desired acceleration [deg/sec^2])
 */
class PredictivePIDFeedbackGenerator
{
private:
  const Trajectory1D *pos_traj;
  const Trajectory1D *vel_traj;
  const Trajectory1D *acc_traj;
  RealTimeDifferentiator differentiator;
  RealTimeIntegrator integrator;
  double P, I, D;
  double vel_factor, acc_factor;

public:
  /**
   * Constructs a new predictive PID feedback generator with the given weight
   * and factor values.
   */
  PredictivePIDFeedbackGenerator(double prop_weight, double integ_weight, double deriv_weight,
  double vel_factor, double acc_factor)
  : pos_traj(nullptr)
  , vel_traj(nullptr)
  , acc_traj(nullptr)
  , P(prop_weight)
  , I(integ_weight)
  , D(deriv_weight)
  , vel_factor(vel_factor)
  , acc_factor(acc_factor)
  {
  }

  /**
   * Resets this generator and gives it a new desired trajectory consisting
   * of its position, velocity, and acceleration.
   */
  void set_trajectory(const Trajectory1D &pos, const Trajectory1D &vel, const Trajectory1D &acc)
  {
    this->pos_traj = &pos;
    this->vel_traj = &vel;
    this->acc_traj = &acc;
    differentiator.reset();
    integrator.reset();
  }

  /**
   * Sets output to the feedback control value generated for the actual position
   * and currently set trajectory. The given time should be the elapsed time
   * since the start of the trajectory since it is used to interpret where the
   * joints should actually be.
   */
  double generate(double time, double position)
  {
    // compute the modified desired position
    double modified_pos =
      pos_traj->value(time) +
      vel_factor*vel_traj->value(time) +
      acc_factor*acc_traj->value(time);

    // find current error (actual - {modified desired})
    double error = position - modified_pos;

    // update error integral
    integrator.input(time, error);

    // update error derivative
    differentiator.input(time, error);

    // compute weighted PID sum and limit to the range of [-100, 100]
    return P*error + I*integrator.integral() + D*differentiator.derivative();
  }
};


/**
 * This class implements full-state feedback control for all six joints of the
 * RASM, each of which is moving along its own trajectory. The full-state
 * control is generated by modifying the poles of a state-space system that
 * was empirically identified.
 *
 * Technical:
 * pole constraints
 * DC gain
 */
class FullStateFeedbackGenerator
{
private:
  const Trajectory6D *pos_traj;

public:
  FullStateFeedbackGenerator(const std::array<std::complex<double>, 6> poles)
  : pos_traj(nullptr)
  {
  }

  /**
   * Resets this generator and gives it a new desired trajectory.
   */
  void set_trajectory(const Trajectory6D &pos)
  {
    this->pos_traj = &pos;
  }

  /**
   * Sets the outputs array to the feedback values generated for the actual
   * positions and currently set trajectory. The given time should be the
   * elapsed time since the start of the trajectory since it is used to
   * interpret where the joints should actually be.
   */
  void generate(double time, const ArrD6 &positions, ArrD6 &outputs)
  {
    outputs.fill(0);
  }
};

#endif
