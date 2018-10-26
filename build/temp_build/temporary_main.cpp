#include "../../include/vision/pose_estimation.hpp"
#include <iostream>
int main(int argc, char** argv)
{
  CameraImageBuffer camera(1);
  FacePoseEstimator faceTester(camera, "/home/browse/Documents/fall_2018/me4010/rasm-v2/test/filesysroot/rasm_2_1/data/vision/prediction/face_model_68_points.dat");
  MarkerPoseEstimator markerTester(camera, "/home/browse/Documents/fall_2018/me4010/rasm-v2/test/filesysroot/rasm_2_1/data/vision/detection/images/detector.svm", "/home/browse/Documents/fall_2018/me4010/rasm-v2/test/filesysroot/rasm_2_1/data/vision/detection/images/sp.dat");
  if(argv[1] == "--marker")
  {
    while(true)
    {
      // ################ WARNING: MAGIC NUMBER! #####################
      std::array<double, 6> pose = markerTester.get_pose();
      std::cout << "\n\nBegin:" << std::endl;
      for(int i = 0; i < pose.size(); i++){
        std::cout << pose[i] << std::endl;
      }
      std::cout << "End" << std::endl;
    }
  }
  else
  {
    while(true)
    {
      // ################ WARNING: MAGIC NUMBER! #####################
      std::array<double, 6> pose = faceTester.get_pose();
      std::cout << "\n\nBegin:" << std::endl;
      for(int i = 0; i < pose.size(); i++){
        std::cout << pose[i] << std::endl;
      }
      std::cout << "End" << std::endl;
    }
  } 
}
