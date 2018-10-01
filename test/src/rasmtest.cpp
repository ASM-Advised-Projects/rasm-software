/**
 * This file defines the main method for the program that runs all software
 * tests for the RASM's software system.
 */

//#include "battery_tests.hpp"
//#include "configuration_tests.hpp"
//#include "logging_tests.hpp"
//#include "battery_tests.hpp"
//#include "peripheral_tests.hpp"
#include "vision_tests.hpp"
//#include "control_tests.hpp"
//#include "http_tests.hpp"
#include "other_tests.hpp"
#include "lest.hpp"

#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
        *(result++) = item;
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int find(std::vector<string> &string_list, const char *search_str)
{
  string str(search_str);
  for (int i = 0; i < string_list.size(); i++)
    if (string_list[i] == str) return i;
  return -1;
}

int option_type(string &str)
{
  if (str == "--help")
    return 1;
  if (str == "--test")
    return 2;
  if (str == "--active")
    return 3;
  if (str == "--non-active")
    return 4;
  if (str == "--lest")
    return 5;
  return 0;
}

/**
 *
 * rasmtest [--help] [fileroot [--test systems] [--active | --non-active]]
 * all systems: configuration:logging:battery:peripheral:vision:control:http:shell:other
 */
int main(int argc, char **argv)
{
  // help/usage printout
  string usage = "usage: rasmtest [--help] [fileroot [--test systems] [--active | --non-active]]\n"
  "       all systems: configuration:logging:battery:peripheral:vision:control:http:shell:other";

  // move command-line options from argv to options
  std::vector<string> options;
  for (int arg_ind = 1; arg_ind < argc; arg_ind++)  // skip first arg
    options.push_back(argv[arg_ind]);

  // print a help message and exit if --help option is present or if no options are present
  if (argc <= 1 || find(options, "--help") != -1)
  {
    cout << usage << endl;
    return 0;
  }

  // get the fileroot
  if (option_type(options[0]) != 0)
  {
    cout << "First argument must be a file path." << endl;
    return 1;
  }
  string fileroot = options[0];

  // default systems to test (all)
  std::vector<string> systems;
  systems.push_back("configuration");
  systems.push_back("logging");
  systems.push_back("battery");
  systems.push_back("peripheral");
  systems.push_back("vision");
  systems.push_back("control");
  systems.push_back("http");
  systems.push_back("shell");
  systems.push_back("other");

  // get possibly reduced set of systems to test
  int test_ind = find(options, "--test");
  if (test_ind != -1)
  {
    if (options.size() < test_ind+2)
    {
      cout << "A list of subsystems must follow the '--test' option." << endl;
      return 0;
    }
    systems = split(options[test_ind+1], ':');
  }

  // get possible interactive options
  int interactive = 0;  // 0 - both; 1 - only non-interactive; 2 - only interactive
  if (find(options, "--active") != -1)
  {
    if (find(options, "--non-active") != -1)
    {
      cout << "--active and --non-active options can't both be specified." << endl;
      return 1;
    }
    interactive = 2;
  }
  else if (find(options, "--non-active") != -1)
  {
    interactive = 1;
  }

  // fileroot, systems
  // for each subsystem, perform non-interactive tests
  if (interactive != 2)
  {
    char *lest_opts[4] = {(char*)"-c", (char*)"-l", (char*)"-p", (char*)"-v"};
    cout << endl << "**Running non-interactive tests.**" << endl;

    //const lest::test *test_cases = {};
    for (auto iter = systems.begin(); iter != systems.end(); iter++)
    {
      cout << endl;
      cout << "Subsystem: " << (*iter) << endl;
      /*if (*iter == "configuration")
        lest::run(config_tests::nonactive, 4, lest_opts);
      else if (*iter == "logging")
        lest::run(logging_tests::nonactive, 4, lest_opts);
      else if (*iter == "battery")
        lest::run(battery_tests::nonactive, 4, lest_opts);
      else if (*iter == "peripheral")
        lest::run(periph_tests::nonactive, 4, lest_opts);
      else if (*iter == "vision")
        lest::run(vision_tests::nonactive, 4, lest_opts);
      else if (*iter == "control")
        lest::run(control_tests::nonactive, 4, lest_opts);
      else if (*iter == "http")
        lest::run(http_tests::nonactive, 4, lest_opts);
      else */if (*iter == "other")
        lest::run(other_tests::nonactive, 4, lest_opts);
      //else
      //  cout << "Invalid subsystem name '" + *iter + "'." << endl;
    }
  }

  // for each subsystem, perform interactive tests
  if (interactive != 1)
  {
    char *lest_opts[4] = {(char*)"-c", (char*)"-l", (char*)"-p", (char*)"-v"};
    cout << endl << "**Running interactive tests.**" << endl;

    for (auto iter = systems.begin(); iter != systems.end(); iter++)
    {
      cout << endl;
      cout << "Subsystem: " << (*iter) << endl;
      /*if (*iter == "configuration")
        lest::run(config_tests::active);
      else if (*iter == "logging")
        lest::run(logging_tests::active);
      else if (*iter == "battery")
        lest::run(battery_tests::active);
      else if (*iter == "peripheral")
        lest::run(periph_tests::active);
      else if (*iter == "vision")
        lest::run(vision_tests::active, 4, lest_opts);
      else if (*iter == "control")
        lest::run(control_tests::active);
      else if (*iter == "http")
        lest::run(http_tests::active);
      else */if (*iter == "other")
        lest::run(other_tests::active, 4, lest_opts);
      //else
      //  cout << "Invalid subsystem name '" + *iter + "'." << endl;
    }
  }

  CameraImageBuffer imgbuffer(0);
  std::cout << "1" << std::endl;
  imgbuffer.get_bgr_image();
  std::cout << "2" << std::endl;
  imgbuffer.get_gray_image();
  std::cout << "3" << std::endl;
  imgbuffer.get_small_bgr_image();
  std::cout << "4" << std::endl;
  imgbuffer.get_small_gray_image();
  std::cout << "5" << std::endl;

  /*CameraImageBuffer imgbuffer(0);
  std::string path = "/Users/joepollard1/Projects/RASM/Software"
  "/test/filesysroot/rasm_2_1/data/vision/prediction/face_model_68_points.dat";
  FacePoseEstimator pose_estimator(imgbuffer, path);

  Pose pose;
  for (int ord = 0; ord < 10; ord++)
  {
    std::cout << "get_pose" << std::endl;
    pose = pose_estimator.get_pose();
    for (int i = 0; i < pose.size(); i++)
      std::cout << pose[i] << ",";
    std::cout << std::endl;
    usleep(1000*1000);
  }*/
}
