/**
 * Defines the CameraCalibration class.
 */

#ifndef CAMERA_CALIBRATION_INCLUDED
#define CAMERA_CALIBRATION_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/**
 *
 */
class CameraCalibration
{
private:


public:
  /**
   *
   *
   * Instead of returning a success/failure indication, an exception will be
   * thrown if any errors occur or if calibration was unsuccessfull.
   */
  static void run_chessboard_calibration(const std::string &settings_filepath, cv::Mat &cam_matrix, cv::Mat &dist_coeffs)
  {

  }

  /**
   *
   */
  static void save_camera_properties(const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs)
  {

  }

  /**
   *
   */
  static bool load_camera_properties(const std::string &calib_filepath, cv::Mat &cam_matrix, cv::Mat &dist_coeffs)
  {

  }
};

#endif
