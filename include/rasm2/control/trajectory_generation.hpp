/**
 *
 */

#ifndef RASM2_CONTROL_TRAJECTORY_GENERATION_HPP
#define RASM2_CONTROL_TRAJECTORY_GENERATION_HPP

#include "definitions.hpp"
#include "trajectory_structures.hpp"
#include "rasm2/util/pose.hpp"

/**
 * realizable trajectory – all trajectory points are reachable with no singularites (inf joint velocities)
 * decides where to place potential via; guarantees realizable trajectory
 */
class TrajectoryGenerator
{
public:
  /**
   *
   */
  struct TrajectoryParams
  {
    double initial_pos;
    double final_pos;
    double initial_vel;
    double final_vel;
    double initial_acc;
    double final_acc;
    double max_vel;
    double max_acc;
  };

  /**
   *
   */
  TrajectoryGenerator()
  {

  }

  // returns the true final global pose
  Pose generate_trajectory(const Pose &initial_global_pose,
      const Pose &final_global_pose, Trajectory6D &trajectory)
  {

  }

  // forward kinematics
  // joint_to_cartesian
  void joint_to_cartesian(const std::array<double, 6> &joint_positions, Pose &pose)
  {

  }

  // inverse kinematics – returns true if reachable
  // cartesian_to_joint
  bool cartesian_to_joint(const Pose &pose, std::array<double, 6> &joint_positions)
  {

  }

  /*static Trajectory1D trapezoidal_trajectory(const TrajectoryParams &params)
  {

  }

  static Trajectory1D cubic_poly_trajectory(const TrajectoryParams &params)
  {

  }

  static Trajectory1D quintic_poly_trajectory(const TrajectoryParams &params)
  {

  }

  static Trajectory1D optimal_trajectory(const TrajectoryParams &params,
      std::function<double (Trajectory6D &)> cost_function)
  {

  }*/
};

#endif
