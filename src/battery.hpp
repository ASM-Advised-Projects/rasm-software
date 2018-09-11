/**
 * Implements the RASM's battery monitoring subsystem.
 * Defines the BatterySentinel class.
 */

#include <unistd.h>
#ifndef BATTERY_INCLUDED
#define BATTERY_INCLUDED


class BatterySentinel
{
  public:
    void sentinel()
    {
      while(ANY_MOTOR_RUNNING)
      {
        sleep(1); //The documentation says to sleep for a few seconds, but 
                  //wouldn't that be an awfully long time?
      }
    }
};

#endif
