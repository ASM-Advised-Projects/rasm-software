cmake_minimum_required(VERSION 2.8.12)
# Every project needs a name.  We call this the "examples" project.
project(create_detector)

add_subdirectory(/home/browse/Documents/fall_2018/me4010/lib/lib/download/dlib-19.13/dlib /home/browse/Documents/fall_2018/me4010/lib/lib/download/dlib-19.13/examples/build/dlib_build)
add_executable(create_detector create_detector.cpp)
target_link_libraries(create_detector dlib::dlib)
