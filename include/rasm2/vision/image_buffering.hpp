/**
 * Defines the CameraImageBuffer class.
 */

#ifndef RASM2_VISION_IMAGE_BUFFERING_HPP
#define RASM2_VISION_IMAGE_BUFFERING_HPP

#include <opencv2/videoio.hpp>

#include "camera_calibration.hpp"
#include "image_filtering.hpp"

/**
 * Instances of this class are single-image buffers that provide different
 * versions of the image like gray scale and reduced size.
 */
class CameraImageBuffer
{
private:
  cv::VideoCapture camera;
  ImageFilter *filter;

  cv::Mat_<cv::Vec3b> bgr_img;
  cv::Mat_<unsigned short> gray_img;
  cv::Mat_<cv::Vec3b> small_bgr_img;
  cv::Mat_<unsigned short> small_gray_img;

  bool have_gray;
  bool have_small_bgr;
  bool have_small_gray;

  int index;
  double downsample_ratio;

  /**
   *
   */
  void initialize()
  {
    if (!camera.isOpened())
    {
      throw std::runtime_error("Camera failed to open.");
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
  CameraImageBuffer(int camera_id, ImageFilter *filter=nullptr)
  : filter(filter)
  {
    camera.open(camera_id);
    initialize();
  }

  /**
   *
   */
  CameraImageBuffer(const std::string &camera_filepath, ImageFilter *filter=nullptr)
  : filter(filter)
  {
    camera.open(camera_filepath);
    initialize();
  }

  /**
   * Grabs the next image in the camera file and applies a filter if one was
   * given during construction. This method will overwrite the previously set
   * image. Note that this method is internally called upon construction.
   */
  void update_image()
  {
    bool retake;
    do
    {
      camera >> bgr_img;

      // apply image filter if it exists
      if (filter != nullptr)
        (*filter)(bgr_img, retake);
      else
        retake = false;
    } while (retake);

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
    width = camera.get(cv::CAP_PROP_FRAME_WIDTH);
    height = camera.get(cv::CAP_PROP_FRAME_HEIGHT);
  }

  /**
   *
   */
  double get_downsample_ratio()
  {
    return downsample_ratio;
  }

  /**
   * Sets the preferred frame size for the images captured by the camera.
   *
   */
  void set_preferred_framesize(unsigned int width, unsigned int height)
  {
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
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
   * Returns th
   */
  const cv::Mat_<cv::Vec3b> & get_bgr_image()
  {
    return bgr_img;
  }

  /**
   *
   */
  const cv::Mat_<unsigned short> & get_gray_image()
  {
    if (!have_gray)
    {
      if(bgr_img.empty())
      {
        std::cerr << "The image is empty!" << std::endl;
      }
      cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
      have_gray = true;
    }
    return gray_img;
  }

  /**
   * Returns a scaled down bgr version of the current image. The amount of
   * scaling is set by the set_downsample_ratio method.
   */
  const cv::Mat_<cv::Vec3b> & get_small_bgr_image()
  {
    if (!have_small_bgr)
    {
      cv::resize(bgr_img, small_bgr_img, cv::Size(), 1.0/downsample_ratio, 1.0/downsample_ratio);
      have_small_bgr = true;
    }
    return small_bgr_img;
  }

  /**
   * Returns a scaled down grayscale version of the original image. The amount
   * of scaling is set by the set_downsample_ratio method.
   */
  const cv::Mat_<unsigned short> & get_small_gray_image()
  {
    if (!have_small_gray)
    {
      if (!have_gray)
      {
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
        have_gray = true;
      }
      cv::resize(gray_img, small_gray_img, cv::Size(), 1.0/downsample_ratio, 1.0/downsample_ratio);
      have_small_gray = true;
    }
    return small_gray_img;
  }
};

#endif
