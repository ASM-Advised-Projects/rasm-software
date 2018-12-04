/**
 * Defines the Trajectory1D and Trajectory6D classes.
 */

#ifndef RASM2_CONTROL_TRAJECTORY_CONTAINERS_HPP
#define RASM2_CONTROL_TRAJECTORY_CONTAINERS_HPP

#include <vector>
#include <array>
#include <functional>

#include "rasm2/control/joint.hpp"

/**
 * This class represents a 1-dimensional trajectory who's value is a function
 * of time. The trajectory starts at a time of 0 and can be scaled by a speed
 * factor.
 */
class Trajectory1D
{
private:
  std::vector< std::function<double (double)> > segments;
  std::vector<double> end_times;
  double speed_factor;
  bool reversed;

public:
  Trajectory1D()
  : speed_factor(1.0)
  , reversed(false)
  {
    std::function<double (double)> zero_segment = [](double){return 0;};
    add_segment(zero_segment, 0);
  }

  void add_segment(const std::function<double (double)> &segment, double duration)
  {
    segments.push_back(segment);

    int last_time = end_times.size() == 0 ? 0 : end_times[end_times.size()];
    end_times.push_back(last_time + duration);
  }

  void append_trajectory(const Trajectory1D &traj1)
  {
    for (int i = 1; i < traj1.segments.size(); ++i)
      add_segment(traj1.segments[i], traj1.end_times[i] - traj1.end_times[i-1]);
  }

  double value(double time) const
  {
    int ssize = segments.size();
    int etsize = end_times.size();

    if (reversed)
      time = end_times[etsize-1] - time;

    if (time < 0)
      time = 0;

    for (int i = 1; i < ssize; ++i)
    {
      if (speed_factor*time < end_times[i])
        return segments[i](speed_factor*(time-end_times[i-1]));
    }

    if (etsize > 1)
      return segments[ssize-1](end_times[etsize-1]-end_times[etsize-2]);
    return 0;
  }

  void set_speed_factor(double factor)
  {
    speed_factor = factor;
  }

  void reverse()
  {
    reversed = !reversed;
  }

  double duration() const
  {
    if (segments.size() > 0)
      return end_times[end_times.size()-1]/speed_factor;
    return 0;
  }

  void clear()
  {
    segments.clear();
    end_times.clear();
    speed_factor = 1.0;
  }
};


/**
 * This class represents a collection of six trajectories, one for each joint
 * of the RASM. The trajectories can be scaled to be the same length as the
 * longest trajectory.
 */
class Trajectory6D
{
private:
  std::array<Trajectory1D, 6> trajectories;

public:
  Trajectory6D()
  {
  }

  void set_trajectory(Joint joint, Trajectory1D &traj1)
  {
    trajectories[(int)joint] = traj1;
  }

  void append_trajectory(const Trajectory6D &traj6)
  {
    for (int j = 0; j < 6; ++j)
      trajectories[j].append_trajectory(traj6.trajectories[j]);
  }

  Trajectory1D & get_trajectory(Joint joint)
  {
    return trajectories[(int)joint];
  }

  double value(Joint joint, double time) const
  {
    return trajectories[(int)joint].value(time);
  }

  void reverse()
  {
    for (auto iter = trajectories.begin(); iter != trajectories.end(); ++iter)
      iter->reverse();
  }

  void autoscale()
  {
    double max_duration = duration();
    for (auto iter = trajectories.begin(); iter != trajectories.end(); ++iter)
      iter->set_speed_factor(iter->duration()/max_duration);
  }

  double duration() const
  {
    double max = 0;
    double time;
    for (auto iter = trajectories.begin(); iter != trajectories.end(); ++iter)
    {
      time = iter->duration();
      if (time > max)
        max = time;
    }
    return max;
  }

  void clear()
  {
    for (int j = 0; j < 6; ++j)
      trajectories[j].clear();
  }
};

#endif
