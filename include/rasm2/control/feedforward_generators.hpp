/**
 * Defines the NaiveFeedforwardGenerator and SSFeedforwardGenerator classes.
 */

#ifndef RASM2_CONTROL_FEEDFORWARD_GENERATORS_HPP
#define RASM2_CONTROL_FEEDFORWARD_GENERATORS_HPP

#include <array>
#include <cmath>

#include "trajectory_containers.hpp"
#include "rasm2/util/differentiators.hpp"

#define PI 3.14159265359

/**
 * This class implements feedforward control for all six joints of the RASM,
 * each of which is moving along its own trajectory. The feedforward control is
 * is generated using a series of equations with weighted terms that were more
 * or less 'derived' from basic link kinetic models and intuition.
 */
class NaiveFeedforwardGenerator
{
private:
  const Trajectory6D *pos_traj;
  const Trajectory6D *vel_traj;
  const Trajectory6D *acc_traj;

public:
  /**
   * Constructs a new feedforward generator.
   */
  NaiveFeedforwardGenerator()
  : pos_traj(nullptr)
  , vel_traj(nullptr)
  , acc_traj(nullptr)
  {
  }

  /**
   * Resets this generator by giving it a new desired trajectory.
   */
  void set_trajectory(const Trajectory6D &pos, const Trajectory6D &vel, const Trajectory6D &acc)
  {
    this->pos_traj = &pos;
    this->vel_traj = &vel;
    this->acc_traj = &acc;
  }

  /**
   * Sets the motor_pwms array to the feedforward duty cycle values generated
   * for the current trajectory. The given time should be the elapsed time
   * since the start of the trajectory since it is used to interpret where the
   * joints should actually be.
   */
  void generate(double time, std::array<float, 6> &motor_pwms)
  {
    // required positions [radians]
    double elbow_pos = PI/180 * pos_traj->value(Joint::ELBOW, time);

    // required velocities [deg/sec]
    double base_vel = vel_traj->value(Joint::BASE, time);
    double shoulder_vel = vel_traj->value(Joint::SHOULDER, time);
    double elbow_vel = vel_traj->value(Joint::ELBOW, time);
    double yaw_vel = vel_traj->value(Joint::WRIST_YAW, time);
    double pitch_vel = vel_traj->value(Joint::WRIST_PITCH, time);
    double roll_vel = vel_traj->value(Joint::WRIST_ROLL, time);

    // required accelerations [deg/sec^2]
    double shoulder_acc = acc_traj->value(Joint::SHOULDER, time);
    double elbow_acc = acc_traj->value(Joint::ELBOW, time);

    motor_pwms[0] = 1.0*base_vel;
    motor_pwms[1] = shoulder_acc*(1.0 + 0.5*cos(elbow_pos))
      + 0.2*elbow_acc*cos(elbow_pos) + 0.5*shoulder_vel;
    motor_pwms[2] = 0.25*shoulder_acc*cos(elbow_pos)
      + 0.25*elbow_acc + 0.5*elbow_vel;
    motor_pwms[3] = 1.0*yaw_vel;
    motor_pwms[4] = 1.0*pitch_vel;
    motor_pwms[5] = 1.0*roll_vel;
  }
};


/**
 * This class implements feedforward control for all six joints of the RASM,
 * each of which is moving along its own trajectory. The feedforward control is
 * is generated using an inverted state-space system that was empirically
 * identified.
 */
class SSFeedforwardGenerator
{
private:
  const Trajectory6D *pos_traj;
  const Trajectory6D *vel_traj;
  const Trajectory6D *acc_traj;

public:
  /**
   * Constructs a new feedforward generator.
   */
  SSFeedforwardGenerator()
  : pos_traj(nullptr)
  , vel_traj(nullptr)
  , acc_traj(nullptr)
  {
  }

  /**
   * Resets this generator by giving it a new desired trajectory.
   */
  void set_trajectory(const Trajectory6D &pos, const Trajectory6D &vel, const Trajectory6D &acc)
  {
    this->pos_traj = &pos;
    this->vel_traj = &vel;
    this->acc_traj = &acc;
  }

  /**
   * Sets the outputs array to the feedforward duty cycle values generated for
   * the currently set trajectory. The given time should be the elapsed time
   * since the start of the trajectory since it is used to interpret where the
   * joints should actually be.
   */
  void generate(double time, std::array<float, 6> &outputs)
  {
    outputs.fill(0);
  }
};

#endif
