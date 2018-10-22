/*
 * Defines the FacePoseEstimator and MarkerPoseEstimator classes.
 */

#ifndef POSE_ESTIMATION_INCLUDED
#define POSE_ESTIMATION_INCLUDED

#include "pose_estimation_base.hpp"


/**
 * Tracks and estimates the pose of human faces.
 */
class FacePoseEstimator : public TrackingPoseEstimator<dlib::scan_fhog_pyramid<dlib::pyramid_down<6>>>
{
private:
  dlib::frontal_face_detector face_detector;
  dlib::shape_predictor face_pose_model;
  std::vector<cv::Point3d> object_pts;
  std::vector<int> image_pt_indices;

protected:
  dlib::object_detector<dlib::scan_fhog_pyramid<dlib::pyramid_down<6>>> * get_object_detector()
  {
    return &face_detector;
  }

  dlib::shape_predictor * get_shape_predictor()
  {
    return &face_pose_model;
  }

  std::vector<cv::Point3d> * get_object_points()
  {
    return &object_pts;
  }

  std::vector<int> * get_image_point_indices()
  {
    return &image_pt_indices;
  }

public:
  FacePoseEstimator(CameraImageBuffer &camera, const std::string &pose_model_filepath)
  : TrackingPoseEstimator(camera)
  {
    face_detector = dlib::get_frontal_face_detector();
    dlib::deserialize(pose_model_filepath) >> face_pose_model;

    std::array<cv::Point3d, 14> temp_pts = {
      cv::Point3d(6.825897, 6.760612, 4.402142),     //#33 left brow left corner
      cv::Point3d(1.330353, 7.122144, 6.903745),     //#29 left brow right corner
      cv::Point3d(-1.330353, 7.122144, 6.903745),    //#34 right brow left corner
      cv::Point3d(-6.825897, 6.760612, 4.402142),    //#38 right brow right corner
      cv::Point3d(5.311432, 5.485328, 3.987654),     //#13 left eye left corner
      cv::Point3d(1.789930, 5.393625, 4.413414),     //#17 left eye right corner
      cv::Point3d(-1.789930, 5.393625, 4.413414),    //#25 right eye left corner
      cv::Point3d(-5.311432, 5.485328, 3.987654),    //#21 right eye right corner
      cv::Point3d(2.005628, 1.409845, 6.165652),     //#55 nose left corner
      cv::Point3d(-2.005628, 1.409845, 6.165652),    //#49 nose right corner
      cv::Point3d(2.774015, -2.080775, 5.048531),    //#43 mouth left corner
      cv::Point3d(-2.774015, -2.080775, 5.048531),   //#39 mouth right corner
      cv::Point3d(0.000000, -3.116408, 6.097667),    //#45 mouth central bottom corner
      cv::Point3d(0.000000, -7.415691, 4.070434)     //#6 chin corner
    };
    object_pts = std::vector<cv::Point3d>(temp_pts.begin(), temp_pts.end());

    std::array<int, 14> temp_inds = {
      17,   //#17 left brow left corner
      21,   //#21 left brow right corner
      22,   //#22 right brow left corner
      26,   //#26 right brow right corner
      36,   //#36 left eye left corner
      39,   //#39 left eye right corner
      42,   //#42 right eye left corner
      45,   //#45 right eye right corner
      31,   //#31 nose left corner
      35,   //#35 nose right corner
      48,   //#48 mouth left corner
      54,   //#54 mouth right corner
      57,   //#57 mouth central bottom corner
      8     //#8 chin corner
    };
    image_pt_indices = std::vector<int>(temp_inds.begin(), temp_inds.end());
  }
};


/**
 * Tracks and estimates the pose of a certain type of visual marker.
 */
class MarkerPoseEstimator : public TrackingPoseEstimator<dlib::scan_fhog_pyramid<dlib::pyramid_down<6>>>
{
private:
  dlib::object_detector<dlib::scan_fhog_pyramid<dlib::pyramid_down<6>>> circle_detector;
  dlib::shape_predictor circle_pose_model;
  std::vector<cv::Point3d> object_pts;
  std::vector<int> image_pt_indices;


protected:
  dlib::object_detector<dlib::scan_fhog_pyramid<dlib::pyramid_down<6>>> * get_object_detector()
  {
    return &circle_detector;
  }

  dlib::shape_predictor * get_shape_predictor()
  {
    return &circle_pose_model;
  }

  std::vector<cv::Point3d> * get_object_points()
  {
    return &object_pts;
  }

  std::vector<int> * get_image_point_indices()
  {
    return &image_pt_indices;
  }


public:
  MarkerPoseEstimator(CameraImageBuffer &camera, const std::string &circle_detector_filepath, const std::string &pose_model_filepath)
  : TrackingPoseEstimator(camera)
  {
    dlib::deserialize(circle_detector_filepath) >> circle_detector;
    dlib::deserialize(pose_model_filepath) >> circle_pose_model;

    std::array<cv::Point3d, 4> temp_pts = {
      cv::Point3d(-4,-1,0),     
      cv::Point3d(-4, 1, 0),   
      cv::Point3d(4, 1, 0),   
      cv::Point3d(4, -1, 0)
    };
    object_pts = std::vector<cv::Point3d>(temp_pts.begin(), temp_pts.end());

    std::array<int, 4> temp_inds = {
      1,   
      2,  
      3, 
      4
    };
    image_pt_indices = std::vector<int>(temp_inds.begin(), temp_inds.end());
  }     
};

#endif
