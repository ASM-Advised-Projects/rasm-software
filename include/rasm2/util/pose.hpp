/**
 * Defines the Pose type and PoseOps class.
 */

#ifndef RASM2_CONTROL_POSE_HPP
#define RASM2_CONTROL_POSE_HPP

#include <array>

typedef std::array<double, 6> Pose;

/**
 * A collection of static methods for operating on Pose objects.
 */
class PoseOps
{
public:
  /**
   * Adds the first and second poses together, element-wise, and returns the
   * resultant pose.
   */
  static Pose add(const Pose &first, const Pose &second)
  {
    Pose result;
    for (int i = 0; i < 6; ++i)
      result[i] = first[i] + second[i];
    return result;
  }

  /**
   * Subtracts the second pose from the first pose, element-wise, and returns
   * the resultant pose.
   */
  static Pose subtract(const Pose &first, const Pose &second)
  {
    Pose result;
    for (int i = 0; i < 6; ++i)
      result[i] = first[i] - second[i];
    return result;
  }

  /**
   * Takes the absolute value of each element of the given pose and returns
   * the resultant pose with those values as its elements.
   */
  static Pose abs(const Pose &pose)
  {
    Pose result;
    for (int i = 0; i < 6; ++i)
      result[i] = pose[i] < 0 ? -pose[i] : pose[i];
    return result;
  }

  /**
   * Returns true if any one of the elements of the first pose is greater than
   * its corresponding element in the second pose; returns false otherwise.
   */
  static bool exceeds(const Pose &first, const Pose &second)
  {
    for (int i = 0; i < 6; ++i)
    {
      if (first[i] > second[i])
        return true;
    }
    return false;
  }

  /**
   * Returns true if any one of the elements of the first pose has a magnitude
   * that is greater than the magnitude of its corresponding element in the
   * second pose; returns false otherwise.
   */
  static bool magnitude_exceeds(const Pose &first, const Pose &second)
  {
    for (int i = 0; i < 6; ++i)
    {
      if (first[i] < 0)
      {
        if (second[i] < 0)
        {
          if (first[i] < second[i])
            return true;
        }
        else
        {
          if (-first[i] > second[i])
            return true;
        }
      }
      else
      {
        if (second[i] < 0)
        {
          if (first[i] > -second[i])
            return true;
        }
        else
        {
          if (first[i] > second[i])
            return true;
        }
      }
    }
    return false;
  }

  /**
   * Returns true if all pose elements in the first pose are equal to their
   * corresponding elements in the second pose with a precision of ±1E-6.
   */
  static bool equal(const Pose &first, const Pose &second)
  {
    double precision = 1 / 1000000.0;
    for (int i = 0; i < 6; ++i)
    {
      double diff = first[i] - second[i];
      if (diff < precision && diff > precision)
        return true;
    }
    return false;
  }

  /**
   * Rotates the translational vectors (first three elements) of the first pose
   * into the frame of the second pose given by the second pose's last three
   * elements which are interpreted as extrinsic Euler angles. The pose with the
   * resultant translational vectors will be returned with its last three
   * elements set to 0.
   */
  static Pose rotate_extrinsic(const Pose &first, const Pose &second)
  {
    Pose result;
    result.fill(0);

    // 
  }
};

#endif
