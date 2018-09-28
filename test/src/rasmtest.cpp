/**
 * This file defines the main method for the program that runs all software
 * tests for the RASM's software system.
 */

#include "battery_tests.hpp"
#include "configuration_tests.hpp"
#include "logging_tests.hpp"
#include "http_tests.hpp"
#include "other_tests.hpp"

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

int contains(std::vector<string> &string_list, const char *search_str)
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
 * rasmtest [--help] [fileroot [--test systems] [--active | --non-active] [--lest "options"]]
 * possible systems: configuration:logging:battery:peripheral:vision:control:http:shell:other
 */
int main(int argc, char **argv)
{
  // move command-line options from argv to options
  std::vector<string> options;
  for (int arg_ind = 1; arg_ind < argc; arg_ind++)  // skip first arg
    options.push_back(argv[arg_ind]);

  // print a help message and exit if --help option is present
  if (contains(options, "--help"))
  {
    string help = "rasmtest [--help] [fileroot [--test systems] [--active | --non-active]"
    " [--lest 'option1 ...']]\nall systems: configuration:logging:battery:peripheral:"
    "vision:control:http:shell:other";
    cout << help << endl;
    return 0;
  }

  // get the fileroot
  if (option_type(options[0]) != 0)
  {
    cout << "First argument must by a file path." << endl;
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
  int test_ind = contains(options, "--test");
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
  if (contains(options, "--active"))
  {
    if (contains(options, "--non-active"))
    {
      cout << "--active and --non-active options can't both be specified." << endl;
      return 1;
    }
    interactive = 2;
  }
  else if (contains(options, "--non-active"))
  {
    interactive = 1;
  }

  // get possible lest options
  string lest_options;
  int lest_ind = contains(options, "--lest");
  if (lest_ind != -1)
  {
    if (options.size() < lest_ind+2)
    {
      cout << "A series of lest-specific options surrounded in quotes must"
      " follow the '--lest' option." << endl;
      return 0;
    }
    lest_options = options[lest_ind+1];
  }

  // fileroot, systems, lest_options
  // for each subsystem, perform non-interactive tests
  if (interactive != 2)
  {

  }

  // for each subsystem, perform interactive tests
  if (interactive != 1)
  {

  }
}
