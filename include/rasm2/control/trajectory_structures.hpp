/**
 *
 */

#ifndef RASM2_CONTROL_TRAJECTORY_STRUCTURES_HPP
#define RASM2_CONTROL_TRAJECTORY_STRUCTURES_HPP

#include <vector>
#include <map>

#include "definitions.hpp"

/**
 *
 */
class TrajectorySegment
{
private:
  std::vector<double> times;
  std::vector<double> positions;
  Time2Pos position_func;

public:
  TrajectorySegment(std::vector<double> times, std::vector<double> positions)
  {

  }

  TrajectorySegment(int start_time, int end_time, Time2Pos function)
  {

  }

  double position(int time_ms)
  {

  }

  void set_time_scale(double factor)
  {

  }

  int start_time()
  {

  }

  int end_time()
  {

  }
};


/**
 *
 */
class Trajectory1D
{
private:
  std::vector<TrajectorySegment> segments;

public:
  Trajectory1D()
  {

  }

  void add_segment(TrajectorySegment segment)
  {

  }

  double position(int time_ms)
  {

  }

  void set_time_scale(double factor)
  {

  }

  int start_time()
  {

  }

  int end_time()
  {

  }
};


/**
 *
 */
class Trajectory6D
{
private:
  std::map<Joint, Trajectory1D> trajectories;

public:
  Trajectory6D()
  {

  }

  void set_trajectory(Joint joint, Trajectory1D trajectory)
  {

  }

  double position(Joint joint, int time_ms)
  {

  }

  void autoscale()
  {

  }

  int start_time()
  {

  }

  int end_time()
  {

  }
};

#endif
