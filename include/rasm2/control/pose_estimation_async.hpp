/**
 * Defines the AsyncPoseEstimator class.
 */

#ifndef RASM2_CONTROL_POSE_ESTIMATION_HPP
#define RASM2_CONTROL_POSE_ESTIMATION_HPP

#include <thread>

#include <Poco/Event.h>

#include "rasm2/util/pose.hpp"
#include "rasm2/vision/pose_estimation.hpp"

/**
 * This class is for running face and marker pose estimation within two
 * threads and provides a notification and frequency control interface.
 */
class AsyncPoseEstimator
{
private:
  FacePoseEstimator &face_estimator;
  MarkerPoseEstimator &marker_estimator;

  std::thread face_thread;
  std::thread marker_thread;

  Pose face_pose;
  Pose marker_pose;

  int face_period;
  int marker_period;
  Poco::Event face_period_event;
  Poco::Event marker_period_event;

  bool exit;

  /**
   * Repeatedly updates face_pose every face_period milliseconds. If
   * face_period_event is signaled then the current wait interval is immediately
   * canceled and a new interval the length of face_period is started. If
   * face_period is negative then no pose updates will occur. Whenever face_pose
   * is updated the face_pose_event and general_pose_event events are signaled.
   */
  void estimate_face_pose()
  {
    while (!exit)
    {
      if (face_period > 0)
      {
        bool timeout = face_period_event.tryWait(face_period);
        if (timeout)
        {
          face_pose = face_estimator.get_pose();
          face_pose_event.set();
          general_pose_event.set();
        }
      }
      else
      {
        face_period_event.wait();
      }
    }
  }

  /**
   * Repeatedly updates marker_pose every marker_period milliseconds. If
   * marker_period_event is signaled then the current wait interval is
   * immediately canceled and a new interval the length of marker_period is
   * started. If marker_period is negative then no pose updates will occur.
   * Whenever marker_pose is updated the marker_pose_event and
   * general_pose_event events are signaled.
   */
  void estimate_marker_pose()
  {
    while (!exit)
    {
      if (marker_period > 0)
      {
        bool timeout = marker_period_event.tryWait(marker_period);
        if (timeout)
        {
          marker_pose = marker_estimator.get_pose();
          marker_pose_event.set();
          general_pose_event.set();
        }
      }
      else
      {
        marker_period_event.wait();
      }
    }
  }

public:
  /**
   * This event is signaled by this estimator whenever the face pose is updated.
   */
  Poco::Event face_pose_event;

  /**
   * This event is signaled by this estimator whenever the marker pose is
   * updated.
   */
  Poco::Event marker_pose_event;

  /**
   * This event is signaled by this estimator whenever either the face pose or
   * marker pose is updated.
   */
  Poco::Event general_pose_event;

  /**
   * Constructs a new asyncronous pose estimator.
   */
  AsyncPoseEstimator(FacePoseEstimator &fpe, MarkerPoseEstimator &mpe)
  : face_estimator(fpe)
  , marker_estimator(mpe)
  , face_thread(&AsyncPoseEstimator::estimate_face_pose, this)
  , marker_thread(&AsyncPoseEstimator::estimate_marker_pose, this)
  {
    face_pose.fill(0);
    marker_pose.fill(0);
    face_period = 0;
    marker_period = 0;
    exit = false;
  }

  /**
   * Destructs this pose estimator.
   */
  ~AsyncPoseEstimator()
  {
    exit = true;
    set_face_period(-1);
    set_marker_period(-1);
    face_thread.join();
    marker_thread.join();
  }

  /**
   * Sets the face estimation period to be the given time in milliseconds. If
   * the given time is negative then face estimation is turned off until this
   * method is called again with a positive time.
   */
  void set_face_period(int millis)
  {
    face_period = millis;
    face_period_event.set();
  }

  /**
   * Sets the marker estimation period to be the given time in milliseconds. If
   * the given time is negative then marker estimation is turned off until this
   * method is called again with a positive time.
   */
  void set_marker_period(int millis)
  {
    marker_period = millis;
    marker_period_event.set();
  }

  /**
   * Returns a const reference to the current face pose.
   */
  const Pose & get_face_pose() const
  {
    return face_pose;
  }

  /**
   * Returns a const reference to the current marker pose.
   */
  const Pose & get_marker_pose() const
  {
    return marker_pose;
  }

  /**
   * Calls TrackingPoseEstimator::lock() on the face pose estimator in use.
   */
  void lock_face()
  {
    face_estimator.lock();
  }

  /**
   * Calls TrackingPoseEstimator::lock() on the marker pose estimator in use.
   */
  void lock_marker()
  {
    marker_estimator.lock();
  }

  /**
   * Calls TrackingPoseEstimator::unlock() on the face pose estimator in use.
   */
  void unlock_face()
  {
    face_estimator.unlock();
  }

  /**
   * Calls TrackingPoseEstimator::unlock() on the marker pose estimator in use.
   */
  void unlock_marker()
  {
    marker_estimator.unlock();
  }
};

#endif
