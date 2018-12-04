/**
 * Defines a main method that runs a trajectory tracking demo.
 *
 * If beaglebone is unable to communicate then pre-compute the trajectories and
 * throw the JointStateEstimator, FeedforwardGenerator, and PIDFeedbackGenerator
 * classes into Arduino with a modified track_trajectory function.
 *
 * Collect a series of data samples during each demo with the following info
 * contained in each sample.
 *   time
 *   desired base/shoulder/elbow joint positions
 *   actual base/shoulder/elbow joint positions
 *   current base/shoulder/elbow feedforward control input
 *   current base/shoulder/elbow feedback control input
 *
 * Filter the actual joint positions and control inputs if needed. Translate
 * desired and actual joint positions into desired and actual screen positions.
 * Plot the desired and actual screen positions on one 3D plot. Plot the
 * feedforward and feedback control input for each joint vs time on three
 * separate plots.
 */

#include <iostream>
#include <array>
#include <cmath>
#include <cstring>
#include <unistd.h>

#include "rasm2/control/trajectory_structures.hpp"
#include "rasm2/control/trajectory_generation.hpp"
#include "rasm2/control/trajectory_tracking.hpp"
#include "rasm2/util/time.hpp"

#include "periphery/serial.h"

#define PI 3.14159265359

using std::cout;
using std::endl;

const char *device_file = "/dev/ttyO1";
int baudrate = 115200;
double P = -3.0;
double I = 0.0;
double D = -0.5;

JointStateEstimator joint_estimator;
FeedforwardGenerator ff_generator;
PIDFeedbackGenerator fb_generator(P, I, D);

std::array<int, 6> encoder_outputs;
std::array<double, 6> joint_positions;

serial_t uart_port;

int init_uart_port()
{
  // open uart device file with defaults
  if (serial_open(&uart_port, device_file, baudrate) < 0) {
    std::cerr << "serial_open(): %s\n" << serial_errmsg(&uart_port) << endl;
    return 1;
  }
  return 0;
}

/**
 * Tracks the given trajectory for the given amount of milliseconds.
 */
void track_trajectory(const Trajectory6D &trajectory, int duration_millis)
{
  fb_generator.set_trajectory(trajectory);
  ff_generator.set_trajectory(trajectory);

  long start_millis = ProgramTime::current_millis();
  long current_millis;

  std::array<float, 6> ff_motor_pwms;
  std::array<float, 6> fb_motor_pwms;
  std::array<float, 6> motor_pwms;

  int relative_millis;
  while (ProgramTime::current_millis() - start_millis < duration_millis)
  {
    // get the present encoder outputs
    get_encoder_outputs(encoder_outputs);
    current_millis = ProgramTime::current_millis();
    relative_millis = current_millis - start_millis;

    // update the joint states
    joint_estimator.update(encoder_outputs, current_millis/1000.0);
    joint_estimator.get_positions(joint_positions);

    // get feedforward control output
    ff_generator.generate_pwms(relative_millis/1000.0, ff_motor_pwms);
    ff_motor_pwms.fill(0);

    // get feedback control output
    fb_generator.generate_pwms(relative_millis/1000.0, joint_positions, fb_motor_pwms);

    // combine feedforward and feedback control outputs
    for (int i = 0; i < 6; ++i)
      motor_pwms[i] = ff_motor_pwms[i] + fb_motor_pwms[i];

    // set motor levels
    set_motor_pwms(motor_pwms);

    // display the following data:
    //   relative time in milliseconds (t)
    //   desired joint positions (djp_s, djp_e)
    //   actual joint positions (ajp_s, ajp_e)
    //   feedforward control input (ff_s, ff_e)
    //   feedback control input (fb_s, fb_e)
    // in the following format (without the line break):
    //   (t)\t(djp_s)\t(djp_e)\t(ajp_s)\t(ajp_e)\t(ff_s)\t(ff_e)\t(fb_s)\t(fb_e)
    cout << relative_millis << "\t"
         << trajectory.position(Joint::SHOULDER, relative_millis/1000.0) << "\t"
         << trajectory.position(Joint::ELBOW, relative_millis/1000.0) << "\t"
         << joint_positions[1] << "\t"
         << joint_positions[2] << "\t"
         << ff_motor_pwms[1] << "\t"
         << ff_motor_pwms[2] << "\t"
         << fb_motor_pwms[1] << "\t"
         << fb_motor_pwms[2] << endl;
  }

  // make sure all joints have stopped moving
  motor_pwms.fill(0);
  set_motor_pwms(motor_pwms);
}

/**
 *
 */
int main(int argc, char **argv)
{
  // check the number of arguments
  if (argc != 3)
  {
    std::cerr << "Exactly two command line arguments are required." << endl;
    std::cerr << "1|2|3(angle) 1|2|3(radius)" << endl;  //cw|ccw(elbow_direction)" << endl;
    return 1;
  }

  // get the option for the final angle
  int angle_option = std::atoi(argv[1]);

  // get the option for the final radius
  int radius_option = std::atoi(argv[2]);

  // get the option for the elbow orientation (cw or ccw folding)
  //  - true for cw, false for ccw
  //bool cw_elbow = std::strcmp(argv[3], "cw");

  double ls = .41;  // shoulder link length
  double le = .50;  // elbow link length
  double theta = (angle_option-2) * 60;
  double r = radius_option * (ls + le) / 3;

  // display the arguments with descriptions
  cout << "Arguments: " << endl;
  cout << "  shoulder-to-screen angle: " << angle_option << " (" << theta << ")" << endl;
  cout << "  shoulder-to-screen distance: " << radius_option << " (" << r << ")" << endl;
  //cout << "  positive elbow angle: " << (cw_elbow ? "false" : "true") << endl;

  // initialize uart port
  cout << endl;
  cout << "Initializing uart port... ";
  if (init_uart_port() != 0)
    return 1;
  cout << "done." << endl;

  // test connection to microcontroller
  cout << "Testing for connection... ";
  int attempt = 0;
  while (true)
  {
    ++attempt;
    if (is_connected())
      break;
    if (attempt == 4)
    {
      cout << "unable to connect." << endl;
      return 1;
    }
    usleep(250*1000);
  }
  cout << "connected." << endl;

  /*std::array<float, 6> motor_pwms;
  motor_pwms.fill(0);
  motor_pwms[1] = 0;
  motor_pwms[2] = -20;
  for (int i = 0; i < 20; ++i)
  {set_motor_pwms(motor_pwms);
  usleep(50*1000);}
  return 0;*/

  // transform options into goal joint positions
  // positions are in degrees
  double shoulder_pos = theta - acos((r*r+ls*ls-le*le)/(2*r*ls)) * 180/PI;
  double elbow_pos = (PI - acos((ls*ls+le*le-r*r)/(2*ls*le))) * 180/PI;

  // get accurate initial joint positions
  for (int i = 0; i < 10; ++i)
  {
    get_encoder_outputs(encoder_outputs);
    joint_estimator.update(encoder_outputs, ProgramTime::current_millis()/1000.0);
    usleep(50*1000);
  }
  joint_estimator.get_positions(joint_positions);

  // output the start positions
  cout << endl;
  cout << "Start Position: " << endl;
  cout << "  shoulder angle: " << joint_positions[1] << " degrees" << endl;
  cout << "  elbow angle: " << joint_positions[2] << " degrees" << endl;

  // output the goal positions
  cout << endl;
  cout << "Goal Position: " << endl;
  cout << "  shoulder angle: " << shoulder_pos << " degrees" << endl;
  cout << "  elbow angle: " << elbow_pos << " degrees" << endl;

  // generate a 1D trajectory for each of the three joints
  // positions are in degrees
  TrajectoryGenerator::TrajectoryParams params;
  params.initial_vel = 0;
  //params.initial_acc = 0;
  params.final_vel = 0;
  //params.final_acc = 0;

  // shoulder
  params.initial_pos = joint_positions[1];
  params.final_pos = shoulder_pos;
  params.max_vel = 20;
  params.max_acc = 20;
  //params.max_jerk = ;
  Trajectory1D shoulder_traj;
  TrajectoryGenerator::cubic_poly_segment(params, shoulder_traj);

  // elbow
  params.initial_pos = joint_positions[2];
  params.final_pos = elbow_pos;
  params.max_vel = 20;
  params.max_acc = 20;
  //params.max_jerk = ;
  Trajectory1D elbow_traj;
  TrajectoryGenerator::cubic_poly_segment(params, elbow_traj);

  // form an autoscaled 6D trajectory
  Trajectory6D traj6;
  traj6.set_trajectory(Joint::SHOULDER, shoulder_traj);
  traj6.set_trajectory(Joint::ELBOW, elbow_traj);
  traj6.autoscale();

  // display trajectory status
  cout << endl;
  cout << "Trajectories have been generated." << endl;
  cout << "All trajectory durations are " << traj6.duration() << " seconds" << endl;

  // track trajectory for 1 second longer than the max trajectory duration
  cout << endl << "Begin trajectory tracking? [y/n] ";
  std::string answer;
  while (true)
  {
    std::cin >> answer;
    if (answer == "y")
      break;
    if (answer == "n")
      return 0;
  }

  cout << "Starting trajectory tracking." << endl;
  track_trajectory(traj6, 1000*(traj6.duration() + 1));
  return 0;
}
