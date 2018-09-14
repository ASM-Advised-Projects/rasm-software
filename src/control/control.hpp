/**
 * Defines the Controller class.
 */

#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "pose_estimation.hpp"


/**
 *
 */
class Controller
{
private:
  /**
   * calls move when necessary
   */
  void monitor()
  {

  }

  /**
   *
   */
  void move()
  {

  }

public:
  Controller()
  {

  }

  ~Controller()
  {

  }

  /**
   * vision thread (semaphore controlled)
   * trajectory thread (semaphore controlled) directly
   * control thread (monitor method)
   */
  void start()
  {

  }

  static bool motors_running()
  {

    return false;
  }
};

#endif
