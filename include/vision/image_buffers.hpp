/**
 * Defines the CameraImageBuffer, UndistCameraImageBuffer, and
 * UnblurCameraImageBuffer classes.
 */

#ifndef IMAGE_BUFFERS_INCLUDED
#define IMAGE_BUFFERS_INCLUDED

#include "camera_calibration.hpp"

#include "opencv2/videoio.hpp"

/**
 * An abstract base class for all image filters that qualify if an image is
 * 'good enough' to be processed by the pose estimation software, and
 * potentially transform the image in a certain way.
 */
class ImageFilter
{
public:
  virtual void operator()(cv::Mat_<unsigned short> &image, bool & retake) = 0;
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

  void operator()(cv::Mat_<unsigned short> &image, bool &retake)
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

  void operator()(cv::Mat_<unsigned short> &image, bool &retake)
  {
    // TODO: use laplacian transform to recognize blur
    retake = false;
  }
};


/**
 * Instances of this class are single-image buffers that provide different
 * versions of the image like gray scale and reduced size.
 */
class CameraImageBuffer
{
private:
  cv::VideoCapture camera;
  ImageFilter *filter;

  cv::Mat_<unsigned short> bgr_img;
  cv::Mat_<unsigned short> gray_img;
  cv::Mat_<unsigned short> small_bgr_img;
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
  const cv::Mat_<unsigned short> & get_bgr_image()
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
      cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
      have_gray = true;
    }
    return gray_img;
  }

  /**
   * Returns a scaled down bgr version of the current image. The amount of
   * scaling is set by the set_downsample_ratio method.
   */
  const cv::Mat_<unsigned short> & get_small_bgr_image()
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
