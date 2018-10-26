/**
 * Defines the ImageFilter superclass along with various subclasses.
 */

#ifndef RASM2_VISION_IMAGE_FILTERING_HPP
#define RASM2_VISION_IMAGE_FILTERING_HPP

#include "opencv2/videoio.hpp"

#include "camera_calibration.hpp"

/**
 * An abstract base class for all image filters that qualify if an image is
 * 'good enough' to be processed by the pose estimation software, and
 * potentially transform the image in a certain way.
 */
class ImageFilter
{
public:
  virtual void filter(cv::Mat &image, bool & retake) = 0;
};


/**
 * A filter that applies a transform to correct any image distortion.
 */
class UndistortFilter : ImageFilter
{
private:
  cv::Mat cam_matrix;
  cv::Mat dist_coeffs;
  cv::Mat distorted;

public:
  UndistortFilter(cv::Mat cam_matrix, cv::Mat dist_coeffs)
  {
    this->cam_matrix = cam_matrix;
    this->dist_coeffs = dist_coeffs;
  }

  void filter(cv::Mat &image, bool &retake)
  {
    // TODO: use initUndistortRectifyMap and remap
    retake = false;
  }
};


/**
 * A filter that discardes images that are 'too' blurry.
 */
class UnblurFilter : ImageFilter
{
private:
  double threshold;

public:
  UnblurFilter(double blur_threshold)
  {
    threshold = blur_threshold;
  }

  void filter(cv::Mat &image, bool &retake)
  {
    // TODO: use laplacian transform to recognize blur
    retake = false;
  }
};

#endif
