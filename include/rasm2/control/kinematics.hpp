/**
 * Defines the Kinematics class.
 */

#ifndef RASM2_CONTROL_KINEMATICS_HPP
#define RASM2_CONTROL_KINEMATICS_HPP

#include "rasm2/control/joint.hpp"
#include "rasm2/util/pose.hpp"

/**
 *
 */
class Kinematics
{
public:
  /**
   *
   */
  struct JointLinkProperties
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

    double screen2camera_up_dist;
    double screen2camera_right_dist;

    double base_min;
    double base_max;

    double shoulder_min;
    double shoulder_max;

    double elbow_min;
    double elbow_max;

    double wristyaw_min;
    double wristyaw_max;

    double wristpitch_min;
    double wristpitch_max;

    double wristroll_min;
    double wristroll_max;
  };

private:
  JointLinkProperties properties;
  Pose aligned_pose;

public:
  /**
   * Constructs a new rasm kinematics instance.
   */
  Kinematics(JointLinkProperties &properties, Pose aligned_pose)
  : properties(properties)
  , aligned_pose(aligned_pose)
  {
  }

  void set_aligned_pose(Pose aligned_pose)
  {
    this->aligned_pose = aligned_pose;
  }

  /**
   * Sets min and max to the minimum and maximum positions allowed by the given
   * joint.
   */
  void joint_limits(Joint joint, double &min, double &max)
  {
    switch (joint)
    {
      case BASE:
        min = properties.base_min;
        max = properties.base_max;
        break;
      case SHOULDER:
        min = properties.shoulder_min;
        max = properties.shoulder_max;
        break;
      case ELBOW:
        min = properties.elbow_min;
        max = properties.elbow_max;
        break;
      case WRIST_YAW:
        min = properties.wristyaw_min;
        max = properties.wristyaw_max;
        break;
      case WRIST_PITCH:
        min = properties.wristpitch_min;
        max = properties.wristpitch_max;
        break;
      case WRIST_ROLL:
        min = properties.wristroll_min;
        max = properties.wristroll_max;
        break;
    }
  }

  /**
   *
   */
  void aligned_screen_pose(const Pose &initial_screen_pose, const Pose &local_object_pose,
  Pose &final_screen_pose)
  {
    Pose rotated_aligned_pose = PoseOps::rotate_extrinsic(aligned_pose, local_object_pose);
    final_screen_pose = PoseOps::add(initial_screen_pose, local_object_pose);
    final_screen_pose = PoseOps::subtract(final_screen_pose, rotated_aligned_pose);
  }

  /**
   * Transforms the joint_positions array into a screen pose using forward
   * kinematics. The screen_pose argument is set equal to the calculated screen
   * pose.
   */
  void joints_to_pose(const ArrD6 &joint_positions, Pose &screen_pose)
  {
  }

  /**
   * Sets the joint_positions array equal to the joint positions needed to
   * produce the given screen pose using inverse kinematics. If ccw_elbow is
   * true then the elbow joint will have a positive angle and vic versa. If the
   * screen pose is not achievable then the joint_positions will be set to
   * produce a pose that comes as close as possible to the desired screen pose.
   * Returns the actual screen pose that the set joint positions will result in.
   */
  Pose pose_to_joints(const Pose &screen_pose, ArrD6 &joint_positions, bool ccw_elbow)
  {
    Pose zero_pose = {0, 0, 0, 0, 0, 0};
    return zero_pose;
  }
};

#endif
