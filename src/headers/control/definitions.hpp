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
struct LinkDimensions
{

};

#endif
