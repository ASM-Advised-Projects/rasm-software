/**
 * Defines the CameraImageBuffer, UndistCameraImageBuffer, and
 * UnblurCameraImageBuffer classes.
 */

#ifndef CAMERA_UTILS_INCLUDED
#define CAMERA_UTILS_INCLUDED

#include "camera_calibration.hpp"

#include "opencv2/opencv.hpp"

/**
 *
 */
class CameraImageBuffer
{
protected:
  cv::VideoCapture camera;

  cv::Mat_<cv::Vec3b> bgr_img;
  cv::Mat_<unsigned short> gray_img;
  cv::Mat_<cv::Vec3b> small_bgr_img;
  cv::Mat_<unsigned short> small_gray_img;

  bool have_gray;
  bool have_small_bgr;
  bool have_small_gray;

  int index;
  double downsample_ratio;

private:
  /**
   *
   */
  void initialize()
  {
    if (!camera.isOpened())
    {
      // throw exception
    }

    index = 0;
    downsample_ratio = 2.0;
    have_gray = false;
    have_small_bgr = false;
    have_small_gray = false;

    set_preferred_framesize(1280, 720);
    update_image();
  }

public:
  /**
   *
   */
  CameraImageBuffer(int camera_id)
  {
    camera.open(camera_id);
    initialize();
  }

  /**
   *
   */
  CameraImageBuffer(const std::string &camera_filepath)
  {
    camera.open(camera_filepath);
    initialize();
  }

  /**
   * Grabs the next image in the camera file, overwriting the previously set
   * image. Note that this method is internally called upon construction.
   */
  virtual void update_image()
  {
    camera >> bgr_img;
    index++;
    have_gray = false;
    have_small_bgr = false;
    have_small_gray = false;
  }

  /**
   * Returns a unique integer identifier for the current image.
   */
  int get_image_index()
  {
    return index;
  }

  /**
   *
   */
  void get_framesize(int &width, int &height)
  {
    width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
    height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
  }

  /**
   *
   */
  double get_downsample_ratio()
  {
    return downsample_ratio;
  }

  /**
   *
   */
  void set_preferred_framesize(unsigned int width, unsigned int height)
  {
    camera.set(CV_CAP_PROP_FRAME_WIDTH, width);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  }

  /**
   *
   */
  void set_downsample_ratio(double ratio)
  {
    downsample_ratio = ratio;
    have_small_bgr = false;
    have_small_gray = false;
  }

  /**
   *
   */
  virtual const cv::Mat_<cv::Vec3b> & get_bgr_image()
  {
    return bgr_img;
  }

  /**
   *
   */
  virtual const cv::Mat_<unsigned short> & get_gray_image()
  {
    if (!have_gray)
    {
      cv:cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
      have_gray = true;
    }
    return gray_img;
  }

  /**
   *
   */
  virtual const cv::Mat_<cv::Vec3b> & get_small_bgr_image()
  {
    if (!have_small_bgr)
    {
      cv::resize(bgr_img, small_bgr_img, cv::Size(), 1.0/downsample_ratio, 1.0/downsample_ratio);
      have_small_bgr = true;
    }
    return small_bgr_img;
  }

  /**
   *
   */
  virtual const cv::Mat_<unsigned short> & get_small_gray_image()
  {
    if (!have_small_gray)
    {
      if (!have_gray)
      {
        cv:cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
        have_gray = true;
      }
      cv::resize(gray_img, small_gray_img, cv::Size(), 1.0/downsample_ratio, 1.0/downsample_ratio);
      have_small_gray = true;
    }
    return small_gray_img;
  }
};


/**
 *
 */
class UndistCameraImageBuffer : public CameraImageBuffer
{
protected:
  cv::Mat cam_matrix;
  cv::Mat dist_coeffs;
  cv::Mat distorted;

public:
  UndistCameraImageBuffer(int camera_id, cv::Mat cam_matrix, cv::Mat dist_coeffs)
  : CameraImageBuffer(camera_id)
  {
    this->cam_matrix = cam_matrix;
    this->dist_coeffs = dist_coeffs;
  }

  UndistCameraImageBuffer(const std::string &camera_filepath, cv::Mat cam_matrix, cv::Mat dist_coeffs)
  : CameraImageBuffer(camera_filepath)
  {
    this->cam_matrix = cam_matrix;
    this->dist_coeffs = dist_coeffs;
  }

  /**
   *
   */
  virtual void update_image()
  {
    camera >> distorted;

    // TODO: use initUndistortRectifyMap and remap

    index++;
    have_gray = false;
    have_small_bgr = false;
    have_small_gray = false;
  }

};


/**
 *
 */
class UnblurCameraImageBuffer : public CameraImageBuffer
{
protected:
  double threshold;

public:
  UnblurCameraImageBuffer(int camera_id, double blur_threshold)
  : CameraImageBuffer(camera_id)
  {
    threshold = blur_threshold;
  }

  UnblurCameraImageBuffer(const std::string &camera_filepath, double blur_threshold)
  : CameraImageBuffer(camera_filepath)
  {
    threshold = blur_threshold;
  }
};

#endif
