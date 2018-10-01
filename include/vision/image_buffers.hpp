/**
 * Defines the CameraImageBuffer, UndistCameraImageBuffer, and
 * UnblurCameraImageBuffer classes.
 */

#ifndef IMAGE_BUFFERS_INCLUDED
#define IMAGE_BUFFERS_INCLUDED

#include "camera_calibration.hpp"

#include "opencv2/videoio.hpp"
#include <iostream>

/**
 * An abstract base class for all image filters that qualify if an image is
 * 'good enough' to be processed by the pose estimation software, and
 * potentially transform the image in a certain way.
 */
class ImageFilter
{
public:
  virtual void filter(cv::Mat_<cv::Vec<unsigned short, 3>> &image, bool & retake) = 0;
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

  void filter(cv::Mat_<cv::Vec<unsigned short, 3>> &image, bool &retake)
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

  void filter(cv::Mat_<cv::Vec<unsigned short, 3>> &image, bool &retake)
  {
    // TODO: use laplacian transform to recognize blur
    retake = false;
  }
};


/**
 * Instances of this class are single-image buffers that provide different
 * versions of the image like gray scale and reduced size.
 * This class is not thread safe.
 */
class CameraImageBuffer
{
private:
  cv::VideoCapture camera;
  ImageFilter *filter;

  cv::Mat_<cv::Vec<unsigned short, 3>> bgr_img;
  cv::Mat_<cv::Vec<unsigned short, 1>> gray_img;
  cv::Mat_<cv::Vec<unsigned short, 3>> conversion_img;

  bool have_gray;
  int index;

public:
  /**
   *
   */
  CameraImageBuffer(int camera_id, ImageFilter *filter=nullptr)
  : filter(filter)
  , bgr_img(480, 640)
  , gray_img(480, 640)
  , conversion_img(480, 640)
  {
    camera.open(camera_id);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    if (!camera.isOpened())
      throw std::runtime_error("Camera failed to open.");

    index = 0;
    have_gray = false;

    update_image();
  }

  /**
   * Grabs the next image in the camera file and applies a filter if one was
   * given during construction. This method will overwrite the previously set
   * image. Note that this method is internally called upon construction.
   */
  void update_image()
  {
    bool retake = false;
    do
    {
      camera >> bgr_img;

      // apply image filter if it exists
      if (filter != nullptr)
        filter->filter(bgr_img, retake);
    } while (retake);

    index++;
    have_gray = false;
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
    width = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
  }

  /**
   * Returns th
   */
  const cv::Mat_<cv::Vec<unsigned short, 3>> & get_bgr_image()
  {
    return bgr_img;
  }

  /**
   *
   */
  const cv::Mat_<cv::Vec<unsigned short, 1>> & get_gray_image()
  {
    if (!have_gray)
    {
  std::cout << "hi1" << std::endl;
      cv::cvtColor(bgr_img, conversion_img, cv::COLOR_BGR2GRAY);
  std::cout << "hi2" << std::endl;
      gray_img = conversion_img;
      have_gray = true;
    }
    return gray_img;
  }
};


#endif
