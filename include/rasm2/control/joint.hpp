/**
 * Defines the Joint enumeration and ArrD6 typedef.
 */

#ifndef RASM2_CONTROL_JOINT_HPP
#define RASM2_CONTROL_JOINT_HPP

#include <array>

typedef std::array<double, 6> ArrD6;

/**
 * This enumeration represents all motorized joints of the RASM.
 */
enum Joint
{
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2,
  WRIST_YAW = 3,
  WRIST_PITCH = 4,
  WRIST_ROLL = 5
};

#endif
