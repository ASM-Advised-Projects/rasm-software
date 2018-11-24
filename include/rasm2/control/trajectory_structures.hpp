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
    this->times = times;
    this->positions = positions;
  }

  TrajectorySegment(int start_time, int end_time, Time2Pos function)
  {
    times = std::vector<double>(2);
    positions = std::vector<double>(2);
    times[0] = (double)start_time;
    times[1] = (double)end_time;
    positions[0] = function(start_time);
    positions[1] = function(end_time);
    //position_func = function;
  }

  double position(int time_ms)
  {

  }

  void set_time_scale(double factor)
  {

  }

  int start_time()
  {
    return times[0];
  }

  int end_time()
  {
    return times.back();
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
    segments.push_back(segment);

  }

  double position(int time_ms)
  {

  }

  void set_time_scale(double factor)
  {

  }

  int start_time()
  {
    return segments[0].start_time();

  }

  int end_time()
  {
    return segments.back().end_time();

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
    trajectories[joint] = trajectory;
  }

  double position(Joint joint, int time_ms)
  {

  }

  void autoscale()
  {

  }

  int start_time()
  {
    return trajectories[BASE].start_time();

  }

  int end_time()
  {
    return trajectories[BASE].start_time();
  }
};

#endif
