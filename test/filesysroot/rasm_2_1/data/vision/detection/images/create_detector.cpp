/*

    This program uses dlib to make an object detector
    for 5 concentric circles. In
    particular, we go though the steps to train the kind of sliding window
    object detector first published by Dalal and Triggs in 2005 in the paper
    Histograms of Oriented Gradients for Human Detection.  

    Note that this program executes fastest when compiled with at least SSE2
    instructions enabled.  So if you are using a PC with an Intel or AMD chip
    then you should enable at least SSE2 instructions.  If you are using cmake
    to compile this program you can enable them by using one of the following
    commands when you create the build project:
        cmake path_to_dlib_root/examples -DUSE_SSE2_INSTRUCTIONS=ON
        cmake path_to_dlib_root/examples -DUSE_SSE4_INSTRUCTIONS=ON
        cmake path_to_dlib_root/examples -DUSE_AVX_INSTRUCTIONS=ON
    This will set the appropriate compiler options for GCC, clang, Visual
    Studio, or the Intel compiler.  If you are using another compiler then you
    need to consult your compiler's manual to determine how to enable these
    instructions.  Note that AVX is the fastest but requires a CPU from at least
    2011.  SSE4 is the next fastest and is supported by most current machines.  

    Follow along with the example file fhog_object_detector_ex.cpp to understand how this
    file works.

*/

#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>

#include <iostream>
#include <fstream>

using namespace std;
using namespace dlib;
int main(int argc, char** argv)
{  

    try
    {
        // In this example we are going to train a face detector based on the
        // concentric circles in the test_images directory.  So the first
        // thing we do is load that dataset.  This means you need to supply the
        // path to this faces folder as a command line argument so we will know
        // where it is.
        if (argc != 2)
        {
            cout << "Give the path to the test_images directory as the argument to this" << endl;
            cout << "program. " << endl;
            cout << endl;
            return 0;
        }
 
        const std::string faces_directory = argv[1];


        dlib::array<array2d<unsigned char> > images_train, images_test;
        std::vector<std::vector<rectangle> > face_boxes_train, face_boxes_test;

        load_image_dataset(images_train, face_boxes_train, faces_directory+"/training.xml");
        load_image_dataset(images_test, face_boxes_test, faces_directory+"/testing.xml");

        upsample_image_dataset<pyramid_down<2> >(images_train, face_boxes_train);
        upsample_image_dataset<pyramid_down<2> >(images_test,  face_boxes_test);

        typedef scan_fhog_pyramid<pyramid_down<6> > image_scanner_type;
        image_scanner_type scanner;
        scanner.set_detection_window_size(140, 140);
        structural_object_detection_trainer<image_scanner_type> trainer(scanner);
        trainer.set_num_threads(4);
        
        // The trainer is a kind of support vector machine and therefore has the usual SVM
        // C parameter.  In general, a bigger C encourages it to fit the training data
        // better but might lead to overfitting.  You must find the best C value
        // empirically by checking how well the trained detector works on a test set of
        // images you haven't trained on.  Don't just leave the value set at 1.  Try a few
        // different C values and see what works best for your data.
        trainer.set_c(1);
        // We can tell the trainer to print it's progress to the console if we want.  
        trainer.be_verbose();
        // The trainer will run until the "risk gap" is less than 0.01.  Smaller values
        // make the trainer solve the SVM optimization problem more accurately but will
        // take longer to train.  For most problems a value in the range of 0.1 to 0.01 is
        // plenty accurate.  Also, when in verbose mode the risk gap is printed on each
        // iteration so you can see how close it is to finishing the training.  
        trainer.set_epsilon(0.01);

        object_detector<image_scanner_type> detector = trainer.train(images_train, face_boxes_train);


        // Now that we have a face detector we can test it.  The first statement tests it
        // on the training data.  It will print the precision, recall, and then average precision.
        cout << "training results: " << test_object_detection_function(detector, images_train, face_boxes_train) << endl;
        // However, to get an idea if it really worked without overfitting we need to run
        // it on images it wasn't trained on.  The next line does this.  Happily, we see
        // that the object detector works perfectly on the testing images.
        cout << "testing results:  " << test_object_detection_function(detector, images_test, face_boxes_test) << endl;


        // If you have read any papers that use HOG you have probably seen the nice looking
        // "sticks" visualization of a learned HOG detector.  This next line creates a
        // window with such a visualization of our detector.  It should look somewhat like
        // 5 concentric circles.
        image_window hogwin(draw_fhog(detector), "Learned fHOG detector");



        // Now for the really fun part.  Let's display the testing images on the screen and
        // show the output of the circle detector overlaid on each image.  You will see that
        // it finds all the circles without false alarming on any non-faces.
        image_window win; 
        for (unsigned long i = 0; i < images_test.size(); ++i)
        {
            // Run the detector and get the face detections.
            std::vector<rectangle> dets = detector(images_test[i]);
            win.clear_overlay();
            win.set_image(images_test[i]);
            win.add_overlay(dets, rgb_pixel(255,0,0));
            cout << "Hit enter to process the next image..." << endl;
            cin.get();
        }


        // Like everything in dlib, you can save your detector to disk using the
        // serialize() function.
        serialize("detector.svm") << detector;
    }
    catch (exception& e)
    {
        cout << "\nexception thrown!" << endl;
        cout << e.what() << endl;
    }
}

