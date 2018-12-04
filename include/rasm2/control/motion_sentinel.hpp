/**
 * Defines the MotionSentinel class.
 */

#ifndef RASM2_CONTROL_MOTION_SENTINEL_HPP
#define RASM2_CONTROL_MOTION_SENTINEL_HPP

#include <array>
#include <cmath>

#include "rasm2/control/joint_state_estimator.hpp"
#include "rasm2/control/trajectory_containers.hpp"
#include "rasm2/util/circular_array.hpp"
#include "rasm2/util/time.hpp"

/**
 * This class monitors the motion of a joint and compares it to the trajectory
 * that it's supposed to be following. This sentinel is able to recognize
 * whether or not a joint has noticeable motion and also if there is an
 * external force/torque acting against a joint's motion.
 */
class MotionSentinel
{
private:
  // fields for evaluating if motion exists or not
  CircularArray<double> positions;
  CircularArray<double> times;
  int index250;
  double vel_threshold;

  // fields for evaluating if an external force exists or not
  const Trajectory1D *pos_traj;
  RealTimeDifferentiator poserr_diff;
  RealTimeDifferentiator velerr_diff;
  double erracc_threshold;

  bool _motion_exists;
  bool force_exists;

  void reset()
  {
    positions.clear();
    times.clear();
    index250 = -1;
    _motion_exists = false;
    force_exists = false;
  }

public:
  /**
   * Constructs a new motion sentinel.
   */
  MotionSentinel(double vel_threshold, double error_acc_threshold)
  : vel_threshold(vel_threshold)
  , erracc_threshold(error_acc_threshold)
  , pos_traj(nullptr)
  , positions(25)
  , times(25)
  {
    reset();
  }

  /**
   * Resets this sentinel with a new desired trajectory.
   */
  void set_trajectory(const Trajectory1D &pos)
  {
    this->pos_traj = &pos;
    reset();
  }

  /**
   * Allows this sentinel to update its predictions using the current state of
   * the joint state estimator.
   */
  void update(double time, double new_pos)
  {
    // update positions, times, and index250
    positions.push(new_pos);
    times.push(time);
    index250 = times.size()-1;
    for (int i = 1; i < times.size()-1; ++i)
    {
      if (time - times[i] > 0.250)
      {
        index250 = i;
        break;
      }
    }

    // update the motion conclusion
    _motion_exists = false;
    if (positions[index250]/times[index250] > vel_threshold)
      _motion_exists = true;

    // update the error acceleration
    double pos_error = new_pos - pos_traj->value(time);
    poserr_diff.input(time, pos_error);
    velerr_diff.input(time, poserr_diff.derivative());

    // update the force conclusion
    force_exists = abs(velerr_diff.derivative()) > erracc_threshold;
  }

  /**
   * Returns true if each of the current joint positions have moved less than
   * a 360th of their range over the last 250 milliseconds.
   */
  bool motion_exists() const
  {
    return _motion_exists;
  }

  /**
   * Returns true if any joint position currently has a significant acceleration
   * of deviation from it's nominal trajectory.
   */
  bool external_force_exists() const
  {
    return force_exists;
  }
};

#endif
