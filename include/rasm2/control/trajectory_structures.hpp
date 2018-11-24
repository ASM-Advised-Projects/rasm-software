/**
 *
 */

#ifndef RASM2_CONTROL_TRAJECTORY_STRUCTURES_HPP
#define RASM2_CONTROL_TRAJECTORY_STRUCTURES_HPP

#include <vector>
#include <map>

/**
 *
 */
class TrajectorySegment
{
private:
  std::vector<double> times;
  std::vector<double> positions;
  std::function<double (int)> position_function;

public:
  TrajectorySegment(const std::vector<double> &times, const std::vector<double> &positions)
  {

  }

  TrajectorySegment(int start_time, int end_time, const std::function<double (int)> &time_to_pos)
  {

  }

  double position(int time_millis)
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

  double position(Joint joint, int time_millis)
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
