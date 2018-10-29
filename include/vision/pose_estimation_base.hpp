/**
 * Defines the TrackingPoseEstimator base class.
 */

#ifndef POSE_ESTIMATION_BASE_INCLUDED
#define POSE_ESTIMATION_BASE_INCLUDED

#include "image_buffers.hpp"

#include "dlib/opencv.h"
#include "dlib/image_processing.h"
#include "dlib/image_processing/frontal_face_detector.h"

typedef std::array<double, 6> Pose;
typedef dlib::rectangle Rect;


/**
 * Abstract base class for optimized pose estimators that track a particular
 * object even if there are multiples of that object present in the same image.
 * The type of images used are 16-bit grayscale retrieved from a given
 * CameraImageBuffer instance.
 */
template <typename ImageScannerType>
class TrackingPoseEstimator
{
private:
  const double BOX_EXPAND_RATIO = 1.2;

  CameraImageBuffer &camera;
  dlib::object_detector<ImageScannerType> *detector;
  dlib::shape_predictor *pose_model;

  std::vector<cv::Point3d> *object_pts;
  std::vector<int> *image_pt_inds;

  cv::Mat cam_matrix;
  cv::Mat dist_coeffs;

  std::vector<cv::Point2d> image_pts;

  double center_weight;
  double proximal_weight;
  //double confidence_weight;

  bool locked;
  int object_count;
  Rect last_object_box;

  Rect zero_box;
  Pose zero_pose;

protected:
  virtual dlib::object_detector<ImageScannerType> * get_object_detector()
  {
    return nullptr;
  }

  virtual dlib::shape_predictor * get_shape_predictor()
  {
    return nullptr;
  }

  virtual std::vector<cv::Point3d> * get_object_points()
  {
    return nullptr;
  }

  virtual std::vector<int> * get_image_point_indices()
  {
    return nullptr;
  }

  /**
   *
   */
  void initialize()
  {
    center_weight = 1.0;
    proximal_weight = 1.0;
    locked = false;
    object_count = 0;

    zero_pose.fill(0);

    detector = get_object_detector();
    pose_model = get_shape_predictor();
    object_pts = get_object_points();
    image_pt_inds = get_image_point_indices();
  }

  /*
   *
   */
  virtual std::vector<Rect> detect_objects(cv::Mat &img)
  {
    std::vector<Rect> object_boxes;
    if (locked)
    {
      // try detecting in viscinity of previous object
      Rect search_box(
        BOX_EXPAND_RATIO * last_object_box.left(),
        BOX_EXPAND_RATIO * last_object_box.top(),
        BOX_EXPAND_RATIO * last_object_box.right(),
        BOX_EXPAND_RATIO * last_object_box.bottom()
      );

      cv::Mat subimg = img(cv::Rect(
        search_box.left(),
        search_box.top(),
        search_box.right() - search_box.left(),
        search_box.bottom() - search_box.top()
      ));

      dlib::cv_image<unsigned short> subimg_dlib(subimg);  // dlib image format
      object_boxes = (*detector)(subimg_dlib);
      if (object_boxes.size() > 0)
        return object_boxes;
    }

    dlib::cv_image<unsigned short> img_dlib(img);  // dlib image format
    return (*detector)(img_dlib);
  }

  /**
   *
   */
  virtual Rect select_object(const std::vector<Rect> &object_boxes, int imgwidth, int imgheight)
  {
    if (object_boxes.size() == 0)
      return zero_box;  // no objects are present - return a zero-filled pose vector

    if (object_boxes.size() == 1)
      return object_boxes[0];  // one object is present - return the only object

    // multiple objects are present - choose an object
    double sum;
    const Rect *box;
    int optimal_ind;

    if (!locked)
    {
      // choose the object with the largest centeredness & proximity weighted sum
      double max_sum = -1;
      int centerx = imgwidth/2;
      int centery = imgheight/2;
      for (int box_ind = 0; box_ind < object_boxes.size(); box_ind++)
      {
        sum = 0;
        box = &object_boxes[box_ind];

        // centeredness is proportional to inverse of the box-center to frame-center distance
        // centeredness = 1 for face center at image center; 0 if at image corner
        int xdelta = centerx - (box->left() + box->right())/2;
        int ydelta = centery - (box->top() + box->bottom())/2;
        sum += center_weight * 0.5*(xdelta / centerx + ydelta / centery);

        // proximity is proportional to the inverse square-root of the area
        // proximity = 1 for face area that is half of image area; 0 for no area
        sum += proximal_weight * std::sqrt(2 * centerx * centery) / std::sqrt(box->area());

        if (sum > max_sum)
        {
          max_sum = sum;
          optimal_ind = box_ind;
        }
      }
    }
    else
    {
      // choose the object with the most similar bounding box as the last selected object
      double min_sum = std::numeric_limits<double>::max();
      for (int box_ind = 0; box_ind < object_boxes.size(); box_ind++)
      {
        box = &object_boxes[box_ind];

        sum = 0;
        sum += std::abs(last_object_box.left() - box->left());
        sum += std::abs(last_object_box.right() - box->right());
        sum += std::abs(last_object_box.top() - box->top());
        sum += std::abs(last_object_box.bottom() - box->bottom());

        if (sum < min_sum)
        {
          min_sum = sum;
          optimal_ind = box_ind;
        }
      }
    }

    return object_boxes[optimal_ind];
  }

public:
  /**
   * Constructs a new pose estimator that uses the given CameraImageBuffer
   * instance as it's source of image frames. The camera matrix is approximated
   * by using the image width for f_x and f_y, half image width for c_x, and
   * half image height for c_y.
   */
  TrackingPoseEstimator(CameraImageBuffer &camera)
  : camera(camera)
  , last_object_box(0, 0, 0, 0)
  , zero_box(0, 0, 0, 0)
  {
    int imgw, imgh;
    camera.get_framesize(imgw, imgh);
    double cam_vals[9] = {(double)imgw, 0, (double)imgw/2, 0, (double)imgw, (double)imgh/2, 0, 0, 1};
    cam_matrix = cv::Mat(3, 3, CV_64FC1, cam_vals);

    double dist_vals[5] = {0, 0, 0, 0, 0};
    dist_coeffs = cv::Mat(5, 1, CV_64FC1, dist_vals);

    initialize();
  }

  /**
   * Constructs a new pose estimator that uses the given CameraImageBuffer
   * instance as it's source of image frames.
   */
  TrackingPoseEstimator(CameraImageBuffer &camera, cv::Mat cam_matrix, cv::Mat dist_coeffs)
  : camera(camera)
  , last_object_box(0, 0, 0, 0)
  , zero_box(0, 0, 0, 0)
  {
    this->cam_matrix = cam_matrix;
    this->dist_coeffs = dist_coeffs;
    initialize();
  }

  /**
   * Detects the objects that are in view of the camera and returns the pose of
   * one of those objects. The selected object that the pose will be computed
   * for is the one with the max weighted score based on frame centeredness and
   * camera proximity. These two weights can be set with the set_center_weight
   * and set_proximal_weight methods. If an object has been locked via a call to
   * the lock method then an object will be selected based on the min distance
   * to the last selected object.
   *
   * If there are no identifiable objects within the image frame then a
   * zero-filled pose array will be returned.
   */
  Pose get_pose()
  {
    std::cout << "0" << std::endl;
    // get downsample ratio for scaled down images
    double img_scale = camera.get_downsample_ratio();
    std::cout << "1" << std::endl;

    // get normal and scaled-down grayscale images
//    const cv::Mat &gray_img = camera.get_gray_image();
//    std::cout << "2" << std::endl;
//    const cv::Mat &small_gray_img = camera.get_small_gray_image();
//    std::cout << "3" << std::endl;
//
    // detect objects using the small image
    const cv::Mat_<cv::Vec3b>  &bgr_img = camera.get_image();
    cv::Mat img(bgr_img);  // TODO - figure out how get around making a copy
    std::cout << "4" << std::endl;
    std::vector<Rect> object_boxes = detect_objects(img);
    std::cout << "5" << std::endl;
    object_count = object_boxes.size();
    std::cout << "6" << std::endl;

    // select an object box
    Rect selected_object_box = select_object(object_boxes, bgr_img.size().width, bgr_img.size().height);
    std::cout << "7" << std::endl;
    if (selected_object_box == zero_box)  // no objects were detected
      return zero_pose;

    // update the last object box
    last_object_box = selected_object_box;

    // scale up selected face box
    selected_object_box.set_left(img_scale*selected_object_box.left());
    selected_object_box.set_right(img_scale*selected_object_box.right());
    selected_object_box.set_top(img_scale*selected_object_box.top());
    selected_object_box.set_bottom(img_scale*selected_object_box.bottom());

    // locate features
    dlib::cv_image<unsigned short> gray_img_dlib(bgr_img);  // dlib image format
    std::cout << "8" << std::endl;
    dlib::full_object_detection shape = (*pose_model)(gray_img_dlib, selected_object_box);
    std::cout << "9" << std::endl;

    // load 2d image points to use
    image_pts.clear();
    for (auto iter = image_pt_inds->begin(); iter != image_pt_inds->end(); iter++)
    {
      image_pts.push_back(cv::Point2d(shape.part(*iter).x(), shape.part(*iter).y()));
    }

    // find the pose of the object
    cv::Mat rotation_vec;      // 3x1 (row x col)
    cv::Mat translation_vec;   // 3x1 (row x col)
    cv::solvePnP((*object_pts), image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);
    Pose pose = {
      translation_vec.at<double>(1, 1),
      translation_vec.at<double>(2, 1),
      translation_vec.at<double>(3, 1),
      rotation_vec.at<double>(1, 1),
      rotation_vec.at<double>(2, 1),
      rotation_vec.at<double>(3, 1)
    };

    return pose;
  }

  /**
   * Sets the frame-centeredness weight for selecting an object.
   * The given weight must be greater than or equal to zero.
   * The default weight is 1.0.
   */
  void set_center_weight(double weight)
  {
    if (weight >= 0)
      center_weight = weight;
  }

  /**
   * Sets the camera-proximity weight for selecting an object.
   * The given weight must be greater than or equal to zero.
   * The default weight is 1.0.
   */
  void set_proximal_weight(double weight)
  {
    if (weight >= 0)
      proximal_weight = weight;
  }

  /**
   * 'Locks' onto the object that was last selected according to the center and
   * proximal weights. This makes the only object selection parameter as the
   * minimum object distance from the location of the last selected object. If
   * no object has been identified yet (zero-filled pose vector returned from
   * get_pose) then this method will do nothing.
   */
  void lock()
  {
    if (last_object_box != zero_box)
      locked = true;
  }

  /**
   * Sets the object selection parameters as based on the usual center and
   * proximal weights.
   */
  void unlock()
  {
    locked = false;
  }

  /**
   * Returns the number of objects found in the last image frame that was
   * processed (via a call to get_pose).
   */
  int get_object_count() const
  {
    return object_count;
  }
};

#endif
