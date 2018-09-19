/**
 *
 */

#ifndef _CONTROL_DEFINITIONS_H_
#define _CONTROL_DEFINITIONS_H_

#include <array>
#include <limits>

/**
 *
 */
typedef std::array<double, 6> sixvector;

/**
 *
 */
typedef double (Time2Pos)(int);

/**
 *
 */
enum Joint
{
  BASE,
  SHOULDER,
  ELBOW,
  WRIST_YAW,
  WRIST_PITCH,
  WRIST_ROLL
};

/**
 *
 */
struct TrajectoryParams
{
  double initial_pos;
  double via_pos = std::numeric_limits<double>::quiet_NaN();
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
struct JointLinkKinematics
{

double base_link_min_height;
double base_link_max_height;
double shoulder_link_length;
double elbow_link_length;
double yaw_link_down_length;
double yaw_link_out_length;
double pitch_link_out_length;
double pitch_link_down_length;
double roll_link_out_length;
double roll_link_down_length;

double screen2camera_up_length;
double screen2camera_right_length;

// angle ranges
double shoulder_min_angle;
double shoulder_max_angle;
double elbow_min_angle;
double elbow_max_angle;
double yaw_min_angle;
double yaw_max_angle;
double pitch_min_angle;
double pitch_max_angle;
double roll_min_angle;
double roll_max_angle;
};

#endif
