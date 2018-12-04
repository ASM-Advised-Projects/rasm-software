/**
 * Defines a main method which runs a program that allows for several different
 * RASM tests to be performed. There are a total of four tests.
 *
 * (1) Motor/Encoder Test
 *   Tests if motors and encoders can be controlled and read.
 *   For selected joints: motor can move the joint in both directions and
 *   the encoder can detect the correct sense of position change.
 *
 *   Notes:
 *   The duty cycle limit will be set to 30%.
 *
 * (2) Zero Position Test
 *   Tests the zero position of each joint.
 *   For selected joints: moves joint to its zero position.
 *
 *   Notes:
 *   The checking part of this test is up to the person running the test.
 *
 * (3) Range of Motion Test
 *   Tests the range of motion for each joint.
 *   For selected joints: moves joint across it's entire allowed range.
 *
 *   Notes:
 *   The checking part of this test is up to the person running the test.
 *
 * (4) Camera Test
 *   Tests if the camera is connected and can have images streamed from it.
 */

#include <iostream>
#include <string>
#include <functional>
#include <unistd.h>
#include <cmath>

#include "c-periphery/serial.h"

#include "rasm2/control/kinematics.hpp"
#include "rasm2/control/feedback_generators.hpp"
#include "rasm2/control/joint_state_estimator.hpp"
#include "rasm2/control/trajectory_generation.hpp"
#include "rasm2/periphery/uc_board.hpp"
#include "rasm2/util/time.hpp"
#include "rasm2/vision/pose_estimator.hpp"

using std::cout;
using std::endl;
using std::cin;
using std::string;

serial_t uartport;
std::unique_ptr<UCBoard> ucboard;
std::array< std::unique_ptr<JointStateEstimator>, 6 > joint_estimators;
std::array< std::unique_ptr<PowIDFeedbackGenerator>, 6 > fb_generators;
std::unique_ptr<Kinematics> kinematics;
//std::unique_ptr<PoseEstimator> pose_estimator;

void update_joint_estimate(Joint joint)
{
  double time = ProgramTime::current_millis() / 1000.0;

  int enc_out;
  ucboard->get_encoder_output(joint, enc_out);
  cout << enc_out << endl;
  joint_estimators[(int)joint]->update(enc_out, time);
}

double enc2pos(Joint joint, double enc_out)
{
  switch (joint)
  {
    case BASE:
      return 0;
      break;
    case SHOULDER:
      return enc_out * 360 / 1023.0 + 0;
      break;
    case ELBOW:
      return enc_out * 360 / 1023.0 + 0;
      break;
    case WRIST_YAW:
      return enc_out * 360 / 1023.0 + 0;
      break;
    case WRIST_PITCH:
      return enc_out * 360 / 1023.0 + 0;
      break;
    case WRIST_ROLL:
      return enc_out * 360 / 1023.0 + 0;
      break;
    default:
      return 0;
      break;
  }
}

void position_print_loop()
{
  while (true)
  {
    for (int j = 0; j < 6; ++j)
      update_joint_estimate((Joint)j);

    cout << "B: " << joint_estimators[0]->get_position() << "\t ";
    cout << "S: " << joint_estimators[1]->get_position() << "\t ";
    cout << "E: " << joint_estimators[2]->get_position() << "\t ";
    cout << "Y: " << joint_estimators[3]->get_position() << "\t ";
    cout << "P: " << joint_estimators[4]->get_position() << "\t ";
    cout << "R: " << joint_estimators[5]->get_position() << "\t ";
    cout << endl;
  }
}

void motor_encoder_test()
{
  cout << "\nBeginning motor & encoder test." << endl;

  string input;
  for (int j = 1; j < 6; ++j)
  {
    cout << "Test " << (Joint)j << " joint? [y/n]: ";
    do { cin >> input; }
    while (input != "y" && input != "n");

    if (input == "n")
      continue;

    for (int i = 0; i < 5; ++i)
      update_joint_estimate((Joint)j);

    bool can_move_pos = false;
    bool valid_dir_pos = false;
    bool can_move_neg = false;
    bool valid_dir_neg = false;

    // apply gradually increasing positive pwm level until joint has moved by
    // 5 degrees

    cout << "applying positive pwm level..." << endl;
    double ipos = joint_estimators[j]->get_position();
    double pos;

    for (int pwm = 0; pwm < 30; ++pwm)
    {
      ucboard->set_motor_pwm((Joint)j, pwm);
      usleep(100*1000);
      pos = joint_estimators[j]->get_position();
      if (abs(pos - ipos) >= 5)
      {
        can_move_pos = true;
        valid_dir_pos = pos > ipos;
        break;
      }
    }

    // apply gradually increasing negative pwm level until joint has moved by
    // 5 degrees

    cout << "applying negative pwm level..." << endl;
    ipos = joint_estimators[j]->get_position();

    for (int pwm = 0; pwm < 30; ++pwm)
    {
      ucboard->set_motor_pwm((Joint)j, pwm);
      usleep(100*1000);
      pos = joint_estimators[j]->get_position();
      if (abs(pos - ipos) >= 5)
      {
        can_move_neg = true;
        valid_dir_neg = pos < ipos;
        break;
      }
    }

    // display results

    if (can_move_pos)
    {
      cout << "Joint moves in ";
      if (valid_dir_pos)
        cout << "correct";
      else
        cout << "incorrect";
      cout << " direction with positive pwm level" << endl;
    }
    else
    {
      cout << "Joint either failed to move in positive direction or couldn't be sensed." << endl;
    }

    if (can_move_neg)
    {
      cout << "Joint moves in ";
      if (valid_dir_neg)
        cout << "correct";
      else
        cout << "incorrect";
      cout << " direction with negative pwm level" << endl;
    }
    else
    {
      cout << "Joint either failed to move in negative direction or couldn't be sensed." << endl;
    }
  }
}

void zero_position_test()
{
  //cout << "\nBeginning zero position test." << endl;



}

void range_of_motion_test()
{

}

void camera_test()
{

}

void track_pose_orientation(bool track_face)
{
  pose_estimator.reset(new PoseEstimator(1));

  for (int i = 0; i < 5; ++i)
  {
    update_joint_estimate(Joint::WRIST_YAW);
    update_joint_estimate(Joint::WRIST_PITCH);
    update_joint_estimate(Joint::WRIST_ROLL);
  }

  Pose pose;
  Pose zero_pose = {0, 0, 0, 0, 0, 0};
  Pose aligned_pose = {100, 0, 0, 0, 0, 0};
  Pose pose_thresholds = {10, 10, 10, 5, 5, 5};
  Pose pose_diff;
  while (true)
  {
    pose = track_face ? pose_estimator->get_face_pose() : pose_estimator->get_marker_pose();

    // loop if pose doesn't exist
    if (PoseOps::equal(pose, zero_pose))
      continue;

    // loop if pose difference from aligned is too small
    Pose pose_diff = PoseOps::subtract(pose, aligned_pose);
    if (!PoseOps::exceeds(pose_diff, pose_thresholds))
      continue;

    double roll_delta = pose[3];
    double pitch_delta = 1.0*pose[2] - 1.0*pose[4];
    double yaw_delta = 1.0*pose[1] + 1.0*pose[5];

    double iroll = joint_estimators[(int)Joint::WRIST_ROLL]->get_position();
    double ipitch = joint_estimators[(int)Joint::WRIST_PITCH]->get_position();
    double iyaw = joint_estimators[(int)Joint::WRIST_YAW]->get_position();

    double froll = iroll + roll_delta;
    double fpitch = ipitch + pitch_delta;
    double fyaw = fyaw + yaw_delta;

    TrajectoryGeneration::TrajectoryParams params;
    params.initial_vel = 0;
    params.final_vel = 0;
    params.max_vel = 10;
    params.max_acc = 20;

    params.initial_pos = iyaw;
    params.final_pos = fyaw;
    Trajectory1D yaw_traj;
    TrajectoryGeneration::cubic(params, &yaw_traj, nullptr, nullptr);

    params.initial_pos = ipitch;
    params.final_pos = fpitch;
    Trajectory1D pitch_traj;
    TrajectoryGeneration::cubic(params, &pitch_traj, nullptr, nullptr);

    params.initial_pos = iroll;
    params.final_pos = froll;
    Trajectory1D roll_traj;
    TrajectoryGeneration::cubic(params, &roll_traj, nullptr, nullptr);

    Trajectory6D traj6;
    traj6.set_trajectory(Joint::WRIST_YAW, yaw_traj);
    traj6.set_trajectory(Joint::WRIST_PITCH, pitch_traj);
    traj6.set_trajectory(Joint::WRIST_ROLL, roll_traj);
    traj6.autoscale();

    fb_generators[(int)Joint::WRIST_YAW]->set_trajectory(yaw_traj);
    fb_generators[(int)Joint::WRIST_PITCH]->set_trajectory(pitch_traj);
    fb_generators[(int)Joint::WRIST_ROLL]->set_trajectory(roll_traj);

    double start_time = ProgramTime::current_millis() / 1000.0;
    double current_time = 0;
    double duration = traj6.duration();

    double yaw_pos, pitch_pos, roll_pos;
    double yaw_pwm, pitch_pwm, roll_pwm;
    while (current_time - start_time <= duration + 0.25)
    {
      current_time = ProgramTime::current_millis() / 1000.0;

      update_joint_estimate(Joint::WRIST_YAW);
      update_joint_estimate(Joint::WRIST_PITCH);
      update_joint_estimate(Joint::WRIST_ROLL);

      yaw_pos = joint_estimators[(int)Joint::WRIST_YAW]->get_position();
      pitch_pos = joint_estimators[(int)Joint::WRIST_PITCH]->get_position();
      roll_pos = joint_estimators[(int)Joint::WRIST_ROLL]->get_position();

      yaw_pwm = fb_generators[(int)Joint::WRIST_YAW]->generate(current_time, yaw_pos);
      pitch_pwm = fb_generators[(int)Joint::WRIST_PITCH]->generate(current_time, pitch_pos);
      roll_pwm = fb_generators[(int)Joint::WRIST_ROLL]->generate(current_time, roll_pos);

      ucboard->set_motor_pwm(Joint::WRIST_YAW, yaw_pwm);
      ucboard->set_motor_pwm(Joint::WRIST_PITCH, pitch_pwm);
      ucboard->set_motor_pwm(Joint::WRIST_ROLL, roll_pwm);
    }

    ucboard->set_motor_pwm(Joint::WRIST_YAW, 0);
    ucboard->set_motor_pwm(Joint::WRIST_PITCH, 0);
    ucboard->set_motor_pwm(Joint::WRIST_ROLL, 0);
  }
}

int main(int argc, char **argv)
{
  // init uc board
  string device_file = "/dev/ttyO1";
  int baudrate = 115200;
  serial_open(&uartport, device_file.c_str(), baudrate);
  ucboard.reset(new UCBoard(&uartport, 50));

  int t;
  while (true)
  {
    cout << "ping: " << ucboard->ping(t) << endl;
    usleep(500);
  }

  // init kinematics
  Kinematics::JointLinkProperties properties;
  properties.base_link_min_height = 0;
  properties.base_link_max_height = 0;
  properties.shoulder_link_length = 0;
  properties.elbow_link_length = 0;
  properties.yaw_link_down_length = 0;
  properties.yaw_link_out_length = 0;
  properties.pitch_link_out_length = 0;
  properties.pitch_link_down_length = 0;
  properties.roll_link_out_length = 0;
  properties.roll_link_down_length = 0;
  properties.screen2camera_up_dist = 0;
  properties.screen2camera_right_dist = 0;
  properties.base_min = 0;
  properties.base_max = 0;
  properties.shoulder_min = 0;
  properties.shoulder_max = 0;
  properties.elbow_min = 0;
  properties.elbow_max = 0;
  properties.wristyaw_min = 0;
  properties.wristyaw_max = 0;
  properties.wristpitch_min = 0;
  properties.wristpitch_max = 0;
  properties.wristroll_min = 0;
  properties.wristroll_max = 0;
  Pose aligned_pose = {0, 0, 0, 0, 0, 0};
  kinematics.reset(new Kinematics(properties, aligned_pose));

  // init fb generators
  double P = -3.0;
  double I = 0.0;
  double D = -0.5;
  for (int j = 0; j < 6; ++j)
  {
    joint_estimators[j].reset(new JointStateEstimator(
      std::bind(enc2pos, (Joint)j, std::placeholders::_1)
    ));
    fb_generators[j].reset(new PowIDFeedbackGenerator(0.5, P, I, D));
  }

  std::string input;
  int option;
  while (true)
  {
    cout << "Options:\n"
      << "\t(0) position output loop\n"
      << "\t(1) motor & encoder test\n"
      << "\t(2) zero-position test\n"
      << "\t(3) range-of-motion test\n"
      << "\t(4) camera test\n"
      << "\t(5) track face pose orientation\n"
      << "\t(6) track marker pose orientation\n"
      << "\t(7) quit\n";

    do
    {
      cin >> input;
      if (input == "quit")
        return 0;
      option = std::stoi(input);
    }
    while (option < 0 || option > 7);

    if (option == 0)
      position_print_loop();
    else if (option == 1)
      motor_encoder_test();
    else if (option == 2)
      zero_position_test();
    else if (option == 3)
      range_of_motion_test();
    else if (option == 4)
      camera_test();
    else if (option == 5)
      track_pose_orientation(true);
    else if (option == 6)
      track_pose_orientation(false);
  }
}
