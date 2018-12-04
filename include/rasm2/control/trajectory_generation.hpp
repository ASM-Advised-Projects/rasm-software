/**
 * Defines the TrajectoryGeneration class.
 */

#ifndef RASM2_CONTROL_TRAJECTORY_GENERATION_HPP
#define RASM2_CONTROL_TRAJECTORY_GENERATION_HPP

// standard library
#include <vector>
#include <cmath>

// 3rd party library
//#include <mav_trajectory_generation/polynomial_optimization_linear.h>

// rasm project
#include <rasm2/control/joint.hpp>
#include <rasm2/control/kinematics.hpp>
#include <rasm2/control/trajectory_containers.hpp>
#include <rasm2/util/pose.hpp>

/**
 *
 */
class TrajectoryGeneration
{
private:

public:
  /**
   *
   */
  enum TrajectoryType
  {
    CUBIC,
    QUINTIC,
    CUBIC_BLENDED,
    QUINTIC_BLENDED
  };

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
    double max_jerk;
  };

  /**
   *
   */
  /*static void generate_trajectory(Trajectory6D &traj6, const std::vector<ArrD6> &positions,
    const ArrD6 &ivel, const ArrD6 &fvel, const ArrD6 &iacc, const ArrD6 &facc)
  {
<<<<<<< HEAD
    mav_trajectory_generation::Vertex::Vector vertices;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACC;
    mav_trajectory_generation::Vertex temp_vertex(6);
    for (int i = 0; i < positions.size(); ++i)
    {
      temp_vertex.
    }
=======
    
>>>>>>> 52e53d094c03381d1dce368e257cd91f9aece8ba

    start(dimension), middle(dimension), end(dimension);

<<<<<<< HEAD
    start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
    vertices.push_back(start);
=======
  // returns the true final global pose
  Pose generate_trajectory(const Pose &initial_global_pose,
      const Pose &final_global_pose, Trajectory6D &trajectory)
  {
    //double theta0 = theta_s;
    //double thetatf = theta_f;
>>>>>>> 52e53d094c03381d1dce368e257cd91f9aece8ba

    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
    vertices.push_back(middle);

    end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
    vertices.push_back(end);

    std::vector<double> segment_times;
    const double v_max = 2.0;
    const double a_max = 2.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    mav_trajectory_generation::Segment::Vector segments;
    opt.getSegments(&segments);


    traj6.clear();

    const ArrD6 zero_array = {0, 0, 0, 0, 0, 0};
    if (initial_joint_vel == nullptr) initial_joint_vel = &zero_array;
    if (final_joint_vel == nullptr) final_joint_vel = &zero_array;
    if (initial_joint_acc == nullptr) initial_joint_acc = &zero_array;
    if (final_joint_acc == nullptr) final_joint_acc = &zero_array;

    TrajectoryParams params;
    Trajectory1D traj1;
    for (int j = 0; j < 6; ++j)
    {
      // populate the params struct for this specific dimension
      params.initial_pos = initial_joint_pos->at(j);
      params.final_pos = final_joint_pos->at(j);
      params.initial_vel = initial_joint_vel->at(j);
      params.final_vel = final_joint_vel->at(j);
      params.initial_acc = initial_joint_acc->at(j);
      params.final_acc = final_joint_acc->at(j);

      switch (ttype)
      {
        case CUBIC:
          cubic(params, &traj1, nullptr, nullptr);
          break;
        case QUINTIC:
          quintic(params, &traj1, nullptr, nullptr);
          break;
        case CUBIC_BLENDED:
          cubic_blended(params, &traj1, nullptr, nullptr);
          break;
        case QUINTIC_BLENDED:
          quintic_blended(params, &traj1, nullptr, nullptr);
          break;
      }

      traj6.set_trajectory((Joint)j, traj1);
      traj1.clear();
    }
  }*/

  /**
   *
   */
  /*static void cubic(const TrajectoryParams &params,
  Trajectory1D *pos, Trajectory1D *vel, Trajectory1D *acc)
  {

  }*/

  /**
   *
   */
  /*static void quintic(const TrajectoryParams &params,
  Trajectory1D *pos, Trajectory1D *vel, Trajectory1D *acc)
  {

  }*/

  /**
   *
   */
  /*static void cubic_blended(const TrajectoryParams &params,
  Trajectory1D *pos, Trajectory1D *vel, Trajectory1D *acc)
  {

  }*/

  /**
   *
   */
  /*static void quintic_blended(const TrajectoryParams &params,
  Trajectory1D *pos, Trajectory1D *vel, Trajectory1D *acc)
  {

  }*/

  /**
   * Computes a one-dimensional trajectory with a single cubic polynomial that
   * obeys the given parameters with the exception of initial acceleration, final
   * acceleration, and max jerk. The magnitude of the velocity and acceleration
   * at every point along the trajectory will be less than or equal to the
   * specified maximums. The time duration of the trajectory will be within 100
   * milliseconds of the minimum possible time.
   */
  static void cubic(const TrajectoryParams &params,
  Trajectory1D *pos, Trajectory1D *vel, Trajectory1D *acc)
  {
    // renaming to make equations more concise
    const double &ui = params.initial_pos;
    const double &uf = params.final_pos;
    const double &vi = params.initial_vel;
    const double &vf = params.final_vel;

    // set initial tf to produce a max acceleration
    double tf = -ui + sqrt(ui*ui - 24*(uf-ui)*params.max_acc) / (12*(uf-ui));

    // Perform an initial iterative search (i=0) followed by a binary search
    // for the minimum tf until within 100 ms of the min. The iterative search
    // increases tf in 1.6 second increments until the max valocity is no
    // longer exceeded.

    double a0, a1, a2, a3;
    double t_maxv, max_vel;
    std::array<double, 5> time_deltas = {1.6, 0.8, 0.4, 0.2, 0.1};

    for (int i = 0; i < time_deltas.size(); ++i)
    {
      if (i != 0)
        tf -= time_deltas[i];

      a0 = ui;
      a1 = vi;
      a2 = 3*(uf - ui) / (tf*tf) - (vf + 2*vi) / tf;
      a3 = 2*(ui - uf) / (tf*tf*tf) + (vf + vi) / (tf*tf);

      // find time of max velocity
      t_maxv = -a2/(3*a3);
      if (t_maxv < 0) t_maxv = 0;
      if (t_maxv > tf) t_maxv = tf;

      // find max valocity
      max_vel = fabs(a1 + 2*a2*t_maxv + 3*a3*t_maxv*t_maxv);

      // if max allowable velocity is exceeded ...
      if (max_vel > params.max_vel)
      {
        tf += time_deltas[i];
        if (i == 0)
          i -= 1;
      }
    }

    if (pos != nullptr)
    {
      std::function<double (double)> fn = [=](double t){return a0 + a1*t + a2*t*t + a3*t*t*t;};
      pos->add_segment(fn, tf);
    }

    if (vel != nullptr)
    {
      std::function<double (double)> fn = [=](double t){return a1 + 2*a2*t + 3*a3*t*t;};
      vel->add_segment(fn, tf);
    }

    if (acc != nullptr)
    {
      std::function<double (double)> fn = [=](double t){return 2*a2 + 6*a3*t;};
      acc->add_segment(fn, tf);
    }
  }

  /**
   * Computes a one-dimensional trajectory with a single quintic polynomial that
   * obeys the given parameters. The magnitude of the velocity, acceleration,
   * and jerk at every point along the trajectory will be less than or equal to
   * the specified maximums. The time duration of the trajectory will be within
   * 100 milliseconds of the minimum possible time.
   */
  /*static void quintic(const TrajectoryParams &params,
  Trajectory1D *pos, Trajectory1D *vel, Trajectory1D *acc)
  {
    // renaming to make equations more concise
    const double &ui = params.initial_pos;
    const double &uf = params.final_pos;
    const double &vi = params.initial_vel;
    const double &vf = params.final_vel;
    const double &ai = params.initial_acc;
    const double &af = params.final_acc;

    // Perform an initial iterative search (i=0) followed by a binary search
    // for the minimum tf until within 100 ms of the min. The iterative search
    // increases tf in 1.6 second increments until the max valocity is no
    // longer exceeded.

    double a0, a1, a2, a3, a4, a5;
    double t_maxv, t_maxa, t_maxj;
    double max_vel, max_acc, max_jerk;
    bool valid_vel, valid_acc, valid_jerk;
    valid_vel = valid_acc = valid_jerk = false;
    std::array<double, 5> time_deltas = {1.6, 0.8, 0.4, 0.2, 0.1};

    double tf = 1.6;
    for (int i = 0; i < time_deltas.size(); ++i)
    {
      if (i != 0)
        tf -= time_deltas[i];

      a0 = ui;
      a1 = vi;
      a2 = 0.5*ai;
      a3 = 1/(2*pow(tf,3)) * (20*(uf-ui) - (8*vf+12*vi)*tf - (3*af-ai)*tf*tf);
      a4 = 1/(2*pow(tf,4)) * (30*(ui-uf) + (14*vf+16*vi)*tf + (3*af-2*ai)*tf*tf);
      a5 = 1/(2*pow(tf,5)) * (12*(uf-ui) - 6*(vf+vi)*tf - (af-ai)*tf*tf);

      // check max velocity
      if (!valid_vel)
      {
        t_maxv = 0;
        max_vel = 0;
        valid_vel = max_vel <= params.max_vel;
      }

      // check max acceleration
      if (!valid_acc)
      {
        t_maxa = 0;
        max_acc = 0;
        valid_acc = max_acc <= params.max_acc;
      }

      // check max jerk
      if (!valid_jerk)
      {
        t_maxj = -a4/(5*a5);
        max_jerk = 6*a3 + 24*a4*t_maxj + 60*a5*t_maxj*t_maxj;
        valid_jerk = max_jerk <= params.max_jerk;
      }

      // if any max is exceeded ...
      if (valid_vel && valid_acc && valid_jerk)
      {
        tf += time_deltas[i];
        if (i == 0)
          i -= 1;
      }
    }

    if (pos != nullptr)
    {
      std::function<double (double)> fn = [=](double t)
          {return a0 + a1*t + a2*t*t + a3*t*t*t + a4*pow(t,4) + a5*pow(t,5);};
      pos->add_segment(fn, tf);
    }

    if (vel != nullptr)
    {
      std::function<double (double)> fn = [=](double t)
          {return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*pow(a5,4);};
      vel->add_segment(fn, tf);
    }

    if (acc != nullptr)
    {
      std::function<double (double)> fn = [=](double t)
          {return 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;};
      acc->add_segment(fn, tf);
    }
  }*/
};

#endif
