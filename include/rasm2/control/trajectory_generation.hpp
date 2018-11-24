/**
 *
 */

#ifndef RASM2_CONTROL_TRAJECTORY_GENERATION_HPP
#define RASM2_CONTROL_TRAJECTORY_GENERATION_HPP

#include "definitions.hpp"
#include "trajectory_structures.hpp"

/**
 * realizable trajectory – all trajectory points are reachable with no singularites (inf joint velocities)
 * decides where to place potential via; guarantees realizable trajectory
 */
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(JointLinkKinematics params)
  {
    

  }

  int compute_trajectory(const sixvector &current_pose, const sixvector &final_pose,
      const sixvector &face_pose, Trajectory6D &trajectory)
  {
    //double theta0 = theta_s;
    //double thetatf = theta_f;

  }

  // forward kinematics
  sixvector compute_pose(const sixvector &joint_positions)
  {

  }

  // inverse kinematics – returns true if reachable
  bool compute_joint_positions(const sixvector &pose, sixvector &joint_positions)
  {

  }

  /**
  *
  */
  static Trajectory1D trapezoidal_trajectory(const TrajectoryParams &params)
  {

  }


  /**
  *
  */
  static Trajectory1D cubic_poly_trajectory(const TrajectoryParams &params)
  {

  }


  /**
  *
  */
  static Trajectory1D quintic_poly_trajectory(const TrajectoryParams &params)
  {

  }

//  static Trajectory1D optimal_trajectory(const TrajectoryParams &params, Trajectory2Cost cost_function)
//  {
//
//  }
};

#endif
