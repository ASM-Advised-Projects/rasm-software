/**
 * Defines the TrajectorySentinel, FeedforwardGenerator, and
 * PIDFeedbackGenerator classes.
 */

#ifndef RASM2_CONTROL_TRAJECTORY_TRACKING_HPP
#define RASM2_CONTROL_TRAJECTORY_TRACKING_HPP

#include <array>

#include "trajectory_structures.hpp"

/**
 * This class estimates the position, velocity, and acceleration of all six
 * joints of the RASM.
 */
class JointStateEstimator
{
private:

public:
  /**
   * Updates the estimations using the new encoder_outputs corresponding to
   * the given time in milliseconds.
   */
  void update(const std::array<double, 6> encoder_outputs, long time_millis)
  {
    return;
  }

  /**
   *
   */
  const std::array<double, 6> & get_positions()
  {
    return;
  }

  /**
   *
   */
  const std::array<double, 6> & get_velocities()
  {
    return;
  }

  /**
   *
   */
  const std::array<double, 6> & get_accelerations()
  {
    return;
  }
};

/**
 * This class monitors the motion of all joints and compares them to the
 * trajectory that they are supposed to be following. This sentinel is able to
 * recognize whether or not any of the joints have noticeable motion and also
 * if there is an external force/torque acting against the motion of any of the
 * joints.
 */
class MotionSentinel
{
protected:
  const Trajectory6D *trajectory;

public:
  /**
   * Constructs a new motion sentinel.
   */
  MotionSentinel()
  : trajectory(nullptr)
  {
  }

  /**
   * Resets this sentinel by giving it a new desired trajectory.
   */
  void set_trajectory(const Trajectory6D &trajectory)
  {
    this->trajectory = &trajectory;
  }

  /**
   * Allows this sentinel to update its predictions based on the current state
   * of the joint state estimator.
   */
  void update()
  {
    return;
  }

  /**
   * Returns true if each of the current joint positions have moved less than
   * a 360th of their range over the last 250 milliseconds.
   */
  bool motion_exists()
  {
    return false;
  }

  /**
   * Returns true if any joint position has had a significant acceleration of
   * deviation from it's nominal trajectory over the last 500 milliseconds.
   */
  bool external_force_exists()
  {
    return false;
  }
};

/**
 * This class implements feedforward control for six joints, each moving along
 * their own trajectory. Uses an inverted state space system matrix to generate
 * the nominal motor-pwm output for moving the joints along their trajectories.
 */
class FeedforwardGenerator
{
protected:

public:
  /**
   * Construct a new feedforward generator.
   */
  FeedforwardGenerator()
  {

  }

  /**
   * Sets the motor_pwms array to the feedforward pwm values generated for the
   * the given trajectory. The current_time should be the elapsed time in
   * milliseconds since the start of the trajectory. This is used to interpret
   * where the joints should be.
   */
  void generate_pwms(const Trajectory6D &trajectory, int current_time,
      std::array<float, 6> &motor_pwms)
  {
    motor_pwms.fill(0);
    return;
  }
};

/**
 * This class implements feedback control for six joints, each moving along
 * their own trajectory. Uses PID control to generate the feedback motor-pwm
 * output for moving the joints along their trajectories.
 */
class PIDFeedbackGenerator
{
protected:

public:
  /**
   * Constructs a new PID feedback generator.
   */
  PIDFeedbackGenerator()
  {

  }

  /**
   * Sets the motor_pwms array to the feedback pwm values generated for the
   * the given trajectory and current joint positions. The current_time should
   * be the elapsed time in milliseconds since the start of the trajectory. This
   * is used to interpret where the joints should actually be.
   */
  void generate_pwms(const Trajectory6D &trajectory, int current_time,
      const std::array<double, 6> joint_positions, std::array<float, 6> &motor_pwms)
  {
    motor_pwms.fill(0);
    return;
  }
};

#endif
