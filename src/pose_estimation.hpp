/**
 * This file contains functions for identifying the presence and estimating
 * the pose of markers and human faces with respect to a camera.
 */

#include <string>
#include "opencv2/opencv.hpp"
#include "dlib/image_processing/frontal_face_detector.h"

#ifndef POSE_ESTIMATION_INCLUDED
#define POSE_ESTIMATION_INCLUDED

typedef std::array<double, 6> Pose;
typedef dlib::rectangle Rect;

using std::string;

/**
 *
 */
struct SelectionWeights
{
  double centeredness;
  double proximity;
  double det_confidence;

  SelectionWeights()
  {
    centeredness = 1.0;
    proximity = 1.0;
    det_confidence = 2.0;
  }

  SelectionWeights(const SelectionWeights &weights)
  {
    centeredness = weights.centeredness;
    proximity = weights.proximity;
    det_confidence = weights.det_confidence;
  }
};


/**
 * Estimates the pose of human faces and a markers of a certain appearance.
 */
class PoseEstimator
{
private:
  cv::VideoCapture camera;
  cv::Mat current_image;
  dlib::frontal_face_detector face_detector;

  SelectionWeights selectionWeights;

  bool locked_face;
  bool locked_marker;

  Rect last_face_rect;
  Rect last_marker_rect;

  Pose last_face_pose;
  Pose last_marker_pose;

  int face_count;
  int marker_count;

public:
  PoseEstimator(const std::string &camerapath)
  : locked_face(false)
  , locked_marker(false)
  , last_face_rect(0, 0, 0, 0)
  , last_marker_rect(0, 0, 0, 0)
  , face_count(0)
  , marker_count(0)
  {
    camera.open(camerapath);
    camera.get(cv::VideoCaptureProperties::)
    if (!camera.isOpened())
    {
      // log error message
      // throw exception
    }

    face_detector = dlib::get_frontal_face_detector();

    last_face_pose.fill(0);
    last_marker_pose.fill(0);
  }

  PoseEstimator(const string &camerapath, const SelectionWeights &weights)
  {
    selectionWeights = SelectionWeights(weights);
    this(camerapath);
  }

  /**
   *
   */
  void lock_face()
  {
    locked_face = true;
  }

  /**
   *
   */
  void unlock_face()
  {
    locked_face = false;
  }

  /**
   *
   */
  void lock_marker()
  {
    locked_marker = true;
  }

  /**
   *
   */
  void unlock_marker()
  {
    locked_marker = false;
  }

  /**
   * Grabs the next image in the camera file, overwriting the previously set
   * image. The currently set image is initally empty upon construction, so this
   * method must be called at least once before get_face_pose or get_marker_pose
   * in order to get a valid pose estimation.
   */
  void update_image()
  {
    camera >> current_image;
  }

  /**
   *
   */
  Pose get_face_pose()
  {
    Pose pose;
    pose.fill(0);

    if (current_image.empty())
      return pose;

    // detect all faces
    cv_image<bgr_pixel> wrapped_img(current_image);
    std::vector<Rect> faces = detector(wrapped_img);

    // return a zero-filled pose vector if no faces are present
    if (face.size() == 0)
      return pose;

    // if there is more than one face presesnt then choose a face to find the pose for


    // find the pose of each face.
    std::vector<full_object_detection> shapes;
    for (unsigned long i = 0; i < faces.size(); ++i)
        shapes.push_back(pose_model(cimg, faces[i]));

  }

  /**
   *
   */
  Pose get_marker_pose()
  {

  }

  /**
   *
   */
  Pose get_last_face_pose() const
  {
    return last_face_pose;
  }

  /**
   *
   */
  Pose get_last_marker_pose() const
  {
    return last_marker_pose;
  }

  /**
   *
   */
  int face_count() const
  {
    return face_count;
  }

  /**
   *
   */
  int marker_count() const
  {
    return marker_count
  }

}

#endif
