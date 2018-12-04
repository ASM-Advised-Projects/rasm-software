//      translation_vec.at<double>(1, 1),
//      translation_vec.at<double>(2, 1), 
//      translation_vec.at<double>(3, 1), 
//      rotation_vec.at<double>(1, 1),
//      rotation_vec.at<double>(2, 1), 
//      rotation_vec.at<double>(3, 1)

#include <iostream>
#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <fstream>
#include <unistd.h>
#include <cmath>

//Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
//Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
double K[9] = {7.3530833553043510e+02, 0.0, 320.0, 0.0, 7.3530833553043510e+02, 240.0, 0.0, 0.0, 1.0};
double D[5] = {-2.3528667558034226e-02, 1.3301431879108856e+00, 0.0, 0.0,
    -6.0786673300480434e+00};

int main(int argc, char** argv)
{
  std::ifstream picture_list("pictures.txt");
    if(!picture_list) {
      std::cout << "Cannot open input file.\n";
    return 1;
  }
    std::ofstream results_file;
    std::ofstream results_file_euler;
    results_file.open("results_file.txt");
    results_file_euler.open("results_file_euler.txt");
    //open cam
    cv::VideoCapture cap(1);
    if (!cap.isOpened())
        {
        std::cout << "Unable to connect to camera" << std::endl;
        return EXIT_FAILURE;
        }
    //Load face detection and pose estimation models (dlib).
    dlib::frontal_face_detector detector;
    dlib::shape_predictor predictor;
    if(!strcmp(argv[1], "--marker"))
    {
      std::cout << "The marker will be detected" << std::endl;

      dlib::deserialize("/home/browse/Documents/fall_2018/me4010/rasm-v2/test/filesysroot/rasm_2_1/data/vision/detection/images/detector.svm") >> detector;
      dlib::deserialize("/home/browse/Documents/fall_2018/me4010/rasm-v2/test/filesysroot/rasm_2_1/data/vision/detection/images/sp.dat") >> predictor;
    }
    else
    {
      detector = dlib::get_frontal_face_detector();
      dlib::deserialize("/home/browse/Documents/fall_2018/me4010/rasm-v2/test/filesysroot/rasm_2_1/data/vision/prediction/face_model_68_points.dat") >> predictor;
    }

    //fill in cam intrinsics and distortion coefficients
    cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
    cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

    //fill in 3D ref points(world coordinates), model referenced from http://aifi.isr.uc.pt/Downloads/OpenGL/glAnthropometric3DModel.cpp
    std::vector<cv::Point3d> object_pts;
    if(strcmp(argv[1], "--marker"))
    {
    object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
    object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
    object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
    object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
    object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
    object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
    object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
    object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
    object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
    object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
    object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
    object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
    object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
    object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner
    }
    else
    {
    object_pts.push_back(cv::Point3d(61, 97, 0));
    object_pts.push_back(cv::Point3d(249, 118, 0));
    object_pts.push_back(cv::Point3d(249, 70, 0));
    object_pts.push_back(cv::Point3d(48, 118, 0));
    }
    //2D ref points(image coordinates), referenced from detected facial feature
    std::vector<cv::Point2d> image_pts;

    //result
    cv::Mat rotation_vec;                           //3 x 1
    cv::Mat rotation_mat;                           //3 x 3 R
    cv::Mat translation_vec;                        //3 x 1 T
    cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
    cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

    //reproject 3D points world coordinate axis to verify result pose
    std::vector<cv::Point3d> reprojectsrc;
    reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
    reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
    reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

    //reprojected 2D points
    std::vector<cv::Point2d> reprojectdst;
    reprojectdst.resize(8);

    //temp buf for decomposeProjectionMatrix()
    cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
    cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

    //text on screen
    std::ostringstream outtext;

    //main loop
#ifdef FILES

  while(picture_list) {
    char str[255];
    picture_list.getline(str, 255);  // delim defaults to '\n'
    cv::Mat temp = cv::imread(str);
#else
    while (1)
    {
      //usleep(500000);
        // Grab a frame
        cv::Mat temp;
        cap >> temp;
#endif
        //experimental!!
            // Camera internals
    //double focal_length = temp.cols; // Approximate focal length.
    //cv::Point2d center = cv::Point2d(temp.cols/2,temp.rows/2);
    //cv::Mat cam_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    //cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion



        //experimental!!!
        dlib::cv_image<dlib::bgr_pixel> cimg(temp);

        // Detect faces
        std::vector<dlib::rectangle> faces = detector(cimg);

        // Find the pose of each face
        if (faces.size() > 0)
            {
            //std::cout << "DETECTED " << faces.size() << std::endl << std::endl;
            //track features
            dlib::full_object_detection shape = predictor(cimg, faces[0]);

            //draw features
            for (unsigned int i = 0; i < 68; ++i)
                {
                cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
                }

            //fill in 2D ref points, annotations follow https://ibug.doc.ic.ac.uk/resources/300-W/
            if(strcmp(argv[1], "--marker"))
            {
            image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
            image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
            image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
            image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
            image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
            image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
            image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
            image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
            image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
            image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
            image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
            image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
            image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
            image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner
            }
            else
            {
            
            image_pts.push_back(cv::Point2d(shape.part(0).x(), shape.part(0).y())); //#57 mouth central bottom corner
            image_pts.push_back(cv::Point2d(shape.part(7).x(), shape.part(7).y()));   //#8 chin corner
            image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner
            image_pts.push_back(cv::Point2d(shape.part(9).x(), shape.part(9).y()));   //#8 chin corner
            }

            //calc pose
            cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);

            //reproject
            cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);

            //draw axis
            cv::line(temp, reprojectdst[0], reprojectdst[1], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[1], reprojectdst[2], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[2], reprojectdst[3], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[3], reprojectdst[0], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[4], reprojectdst[5], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[5], reprojectdst[6], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[6], reprojectdst[7], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[7], reprojectdst[4], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[0], reprojectdst[4], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[1], reprojectdst[5], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[2], reprojectdst[6], cv::Scalar(0, 0, 255));
            cv::line(temp, reprojectdst[3], reprojectdst[7], cv::Scalar(0, 0, 255));
            cv::Point2d my_point = ((reprojectdst[0] + reprojectdst[1] + reprojectdst[2] + reprojectdst[3] + reprojectdst[4] + reprojectdst[5] + reprojectdst[6] + reprojectdst[7])/8);
            double my_x = my_point.x;
            double my_y = my_point.y;
            double x_for_pose = my_x/temp.cols;
            double y_for_pose = my_y/temp.rows;
            std::cout << x_for_pose << std::endl;
            std::cout << y_for_pose << std::endl;
            x_for_pose = x_for_pose -0.5;
            y_for_pose = y_for_pose -0.5;
            double x_distance = -translation_vec.at<double>(1,1)*atan(27.6*3.14159265/180)*x_for_pose/0.5;
            double y_distance = translation_vec.at<double>(1,1)*atan(20*3.14159265/180)*y_for_pose/0.5;

            //calc euler angle
            cv::Rodrigues(rotation_vec, rotation_mat);
            cv::hconcat(rotation_mat, translation_vec, pose_mat);
            cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

                          //show angle result
            //outtext << "X: " << std::setprecision(3) << euler_angle.at<double>(0);
            outtext << "Distance from camera [in]: " << std::setprecision(3) << -translation_vec.at<double>(1, 1)/2.54;
            cv::putText(temp, outtext.str(), cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            outtext << "x position [in]: " << std::setprecision(3) << x_distance/2.54;
            //outtext << "Y: " << std::setprecision(3) << euler_angle.at<double>(1);
            cv::putText(temp, outtext.str(), cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "Z: " << std::setprecision(3) << euler_angle.at<double>(2);
            outtext << "y position [in]: " << std::setprecision(3) << y_distance/2.54;
            cv::putText(temp, outtext.str(), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "rot_vec 1: " << std::setprecision(3) << rotation_vec.at<double>(1, 1);
            outtext << "Pitch in degrees: " << std::setprecision(3) << euler_angle.at<double>(0);
            cv::putText(temp, outtext.str(), cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "rot_vec 2: " << std::setprecision(3) << rotation_vec.at<double>(2, 1);
            outtext << "Yaw in degrees: " << std::setprecision(3) << euler_angle.at<double>(1);
            cv::putText(temp, outtext.str(), cv::Point(50, 120), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");
            //outtext << "rot_vec 3: " << std::setprecision(3) << rotation_vec.at<double>(3, 1);
            outtext << "Roll in degrees: " << std::setprecision(3) << euler_angle.at<double>(2);
            cv::putText(temp, outtext.str(), cv::Point(50, 140), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0));
            outtext.str("");


            // Put the results of the image processing into a file so that it can read and
            // compared
            //results_file << std::endl;
            //results_file_euler << std::endl;
            //results_file_euler << std::endl;
#ifdef FILES
            //results_file << str << std::endl;
            //results_file_euler << str << std::endl;
#endif
           //results_file << "trans_vec 1: " << std::setprecision(3) << translation_vec.at<double>(1, 1) << std::endl;
            //std::cout << "trans_vec 1: " << std::setprecision(3) << translation_vec.at<double>(1, 1) << std::endl;
            //results_file << "trans_vec 2: " << std::setprecision(3) << translation_vec.at<double>(2, 1) << std::endl;
            //std::cout << "trans_vec 2: " << std::setprecision(3) << translation_vec.at<double>(2, 1) << std::endl;
            //results_file << "trans_vec 3: " << std::setprecision(3) << translation_vec.at<double>(3, 1) << std::endl;
            //std::cout << "trans_vec 3: " << std::setprecision(3) << translation_vec.at<double>(3, 1) << std::endl;
            //results_file << "rot_vec 1: " << std::setprecision(3) << rotation_vec.at<double>(1, 1) << std::endl;
            //std::cout << "rot_vec 1: " << std::setprecision(3) << rotation_vec.at<double>(1, 1) << std::endl;
            //results_file << "rot_vec 2: " << std::setprecision(3) << rotation_vec.at<double>(2, 1) << std::endl;
            //std::cout << "rot_vec 2: " << std::setprecision(3) << rotation_vec.at<double>(2, 1) << std::endl;
            //results_file << "rot_vec 3: " << std::setprecision(3) << rotation_vec.at<double>(3, 1) << std::endl;
            //std::cout << "rot_vec 3: " << std::setprecision(3) << rotation_vec.at<double>(3, 1) << std::endl;

            std::cout << std::endl;
            //results_file_euler << "X: " << std::setprecision(3) << euler_angle.at<double>(0) << " ";
            std::cout << "X: " << std::setprecision(3) << euler_angle.at<double>(0) << " ";
            //results_file_euler << "Y: " << std::setprecision(3) << euler_angle.at<double>(1) << " ";
            std::cout << "Y: " << std::setprecision(3) << euler_angle.at<double>(1) << " ";
            //results_file_euler << "Z: " << std::setprecision(3) << euler_angle.at<double>(2) << " ";
            std::cout << "Z: " << std::setprecision(3) << euler_angle.at<double>(2) << " ";
            std::cout << std::endl;


            image_pts.clear();
            }

        //press esc to end
        cv::namedWindow( "demo", cv::WINDOW_NORMAL);
        cv::imshow("demo", temp);
        unsigned char key = cv::waitKey(1);
        if (key == 27)
            {
            break;
            }
    }
    picture_list.close();
    results_file.close();
    results_file_euler.close();
    return 0;
}
