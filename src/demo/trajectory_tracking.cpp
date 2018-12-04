/**
 * Defines a main method which runs a program that allows for several different
 * screen trajectories to be carried out.
 */

#include <iostream>
#include <string>

#include "rasm2/control/kinematics.hpp"
#include "rasm2/control/trajectory_containers.hpp"
#include "rasm2/control/trajectory_generation.hpp"
#include "rasm2/util/pose.hpp"

using std::cout;
using std::endl;
using std::cin;

Kinematics *kinematics;

void init_kinematics();

ArrD6 get_joint_positions()
{

}

void track_trajectory(Trajectory6D traj6)
{

}

void forward_facing_line()
{
  const int num_vertices = 11;
  double start_x = 0.4;
  double end_x = 0.8;
  double max_vel = 20;

  std::array<ArrD6, num_vertices> joint_positions_arr;
  joint_positions_arr[0] = get_joint_positions();

  Pose pose = {0, 0, 1.5, 0, 0, 0};
  Trajectory6D temp_traj6;
  Trajectory6D traj6_1;
  Trajectory6D traj6_2;

  // start to end
  for (int vertex = 1; vertex < num_vertices; ++vertex)
  {
    pose[0] = {start_x + (end_x-start_x) * (vertex/(double)num_vertices)};
    kinematics->pose_to_joints(pose, joint_positions_arr[vertex], true);
    TrajectoryGeneration::generate_trajectory(temp_traj6, TrajectoryGeneration::TrajectoryType::CUBIC,
      &joint_positions_arr[vertex-1], &joint_positions_arr[vertex], nullptr, nullptr, nullptr, nullptr);
    traj6_1.append_trajectory(temp_traj6);
  }

  // end to start
  for (int vertex = num_vertices-1; vertex > ; ++vertex)
  {
    pose[0] = {start_x + (end_x-start_x) * (vertex/(double)num_vertices)};
    kinematics->pose_to_joints(pose, joint_positions_arr[vertex], true);
    TrajectoryGeneration::generate_trajectory(temp_traj6, TrajectoryGeneration::TrajectoryType::CUBIC,
      &joint_positions_arr[vertex-1], &joint_positions_arr[vertex], nullptr, nullptr, nullptr, nullptr);
    traj6_2.append_trajectory(temp_traj6);
  }

  track_trajectory(traj6_1);
  track_trajectory(traj6_2);
}

void forward_facing_circle()
{

}

void yaw_pitch_loop()
{

}

void yaw_pitch_roll_loop()
{

}

int main(int argc, char **argv)
{
  init_kinematics();

  std::string input;
  int option;
  while (true)
  {
    cout << "Trajectory options:\n"
      << "\t(1) forward-facing line\n"
      << "\t(2) forward-facing circle\n"
      << "\t(3) yaw-pitch loop\n"
      << "\t(4) yaw-pitch-roll\n"
      << "\t(5) quit\n";

    do
    {
      cin >> input;
      if (input == "quit")
        return 0;
      option = std::stoi(input);
    }
    while (option < 1 || option > 4);

    if (option == 1)
      forward_facing_line();
    else if (option == 2)
      forward_facing_circle();
    else if (option == 3)
      yaw_pitch_loop();
    else if (option == 4)
      yaw_pitch_roll_loop();
  }
}

void init_kinematics()
{

}
