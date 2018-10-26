/**
 * Defines the ImageBuffer class.
 */

#ifndef RASM2_VISION_IMAGE_BUFFERING_OLD_HPP
#define RASM2_VISION_IMAGE_BUFFERING_OLD_HPP

#include "opencv2/videoio.hpp"

#include "camera_calibration.hpp"

/**
 * Instances of this class are single-image buffers that provide different
 * versions of the image like gray scale and reduced size.
 * Instances of this class are not thread safe.
 */
class ImageBuffer
{
public:
  /**
   * This enumeration represents the allowable image sizes coming from the
   * image source used by an instance of this class. Note that most cameras
   * will only be able to do 480p and 720p.
   */
  enum FrameSize { _360P, _480P, _720P, _1080P };

  /**
   * Converts a frame size enumeration value to the width and height in pixels
   * that it represents.
   */
  static void dimensions(FrameSize framesize, int &width, int &height)
  {
    switch (framesize)
    {
      case _360P:
        width = 480;
        height = 360;
        break;
      case _480P:
        width = 858;
        height = 480;
        break;
      case _720P:
        width = 1280;
        height = 720;
        break;
      case _1080P:
        width = 1920;
        height = 1080;
        break;
    }
  }

private:
  cv::VideoCapture source;
  ImageFilter *filter;

  cv::Mat bgr_img;
  cv::Mat gray_img;
  bool have_gray;
  int index;

  /**
   * Holds initialization/construction code that is common to both constructors.
   */
  void init(FrameSize size)
  {
    set_img_size(size);

    if (!source.isOpened())
      throw std::runtime_error("Image source failed to open.");

    index = 0;
    update_image();
  }

public:
  /**
   *
   */
  ImageBuffer(int camera_id, FrameSize size, ImageFilter *filter=nullptr)
  : filter(filter)
  , have_gray(false)
  {
    source.open(camera_id);
    init(size);
  }

  /**
   *
   */
  ImageBuffer(std::string &filepath, FrameSize size, ImageFilter *filter=nullptr)
  : filter(filter)
  {
    source.open(filepath);
    init(size);
  }

  /**
   * Grabs the next image from the image source and applies a filter if one was
   * given during construction. This method will overwrite the previously set
   * image. Note that this method is internally called upon construction.
   */
  void update_image()
  {
    bool retake = false;
    do
    {
      source >> bgr_img;

      // apply image filter if it exists
      if (filter != nullptr)
        filter->filter(bgr_img, retake);
    } while (retake);

    index++;
  }

  /**
   * Returns the current dimensions in pixels of the images coming from the
   * image source.
   */
  void get_img_size(int &width, int &height)
  {
    width = source.get(cv::CAP_PROP_FRAME_WIDTH);
    height = source.get(cv::CAP_PROP_FRAME_HEIGHT);
  }

  /**
   * Sets the frame size of the images coming from the image source. A camera
   * image source may or may not respect this setting so don't assume the
   * images are of this size until verified with the get_img_size method. A
   * file pattern image source will completely ignore this setting.
   */
  void set_img_size(FrameSize framesize)
  {
    int width, height;
    dimensions(framesize, width, height);

    // width and height are switched here because width corresponds to rows
    // and height corresponds to columns in openCV
    source.set(cv::CAP_PROP_FRAME_WIDTH, height);
    source.set(cv::CAP_PROP_FRAME_HEIGHT, width);
  }

  /**
   * Returns a unique integer identifier for the current image.
   */
  int get_image_index()
  {
    return index;
  }

  /**
   * Returns the latest filtered image taken from the image source.
   */
  const cv::Mat & get_bgr_image()
  {
    return bgr_img;
  }

  /**
   * Returns the grayscale version of the latest filtered image taken from the
   * image source.
   */
  const cv::Mat & get_gray_image()
  {
    if (!have_gray)
    {
      cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
      have_gray = true;
    }
    return gray_img;
  }
};

#endif
