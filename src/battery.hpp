/**
 * Implements the RASM's battery monitoring subsystem.
 * Defines the BatterySentinel class.
 */

#include  <cstdlib>
#include <fstream>
#include <string>
#ifndef BATTERY_INCLUDED
#define BATTERY_INCLUDED


class BatterySentinel
{
  public:
    int sentinel()
    {
      std::string line;
      int voltage;
      std::system("echo cape-bone-iio > sys/devices/bone_capemgr.*/slots");
      std::ifstream battery_file("/sys/devices/ocp.2/helper.14/AIN1");
      if(battery_file.is_open())
      {
        getline(battery_file, line);
        voltage << std::stoi(line);
        battery_file.close();
        return voltage;
      }
      throw std::exception();


    }
};

#endif
