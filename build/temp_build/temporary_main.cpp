#include <iostream>
#include "../../../rasm-v2/include/rasm2/vision/pose.hpp"
int main(int argc, char** argv)
{
PoseEstimator estimator(1);
estimator.get_pose_face();
estimator.get_pose_marker();
}
