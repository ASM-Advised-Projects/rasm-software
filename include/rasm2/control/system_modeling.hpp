/**
 *
 */

#ifndef RASM2_CONTROL_SYSTEM_MODELING_HPP
#define RASM2_CONTROL_SYSTEM_MODELING_HPP

#include <vector>

#include "definitions.hpp"

using std::string;

/**
 *
 */
class SystemModeling
{
public:
  /**
  *
  */
  static void generate_data_stepresponses(Joint joint, int sample_freq, const string &filepath)
  {

  }

  /**
  *
  */
  static void generate_data_inputgiven(Joint joint, int sample_freq, const string &filepath, std::vector<double> inputdata)
  {

  }

  /**
  *
  */
  static void generate_data_inputgiven(int sample_freq, const string &filepath, std::vector<sixvector> inputdata)
  {

  }
};


#endif
