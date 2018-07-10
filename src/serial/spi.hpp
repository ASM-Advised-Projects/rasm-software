/**
 * Defines the SPIDevice class.
 */

#ifndef SPI_INCLUDED
#define SPI_INCLUDED

#include <string>

/**
 *
 */
class SPIDevice
{
private:


public:
  /**
   *
   */
  enum ParityMode { NONE, ODD, EVEN };

  /**
   *
   */
  enum IdleLevel { LOW, HIGH };

  /**
   *
   */
  enum CaptureEdge { RISING, FALLING };

  /**
  *
  */
  struct Settings
  {
    std::string filepath;  // absolute path to device file
    IdleLevel idle_mode;
    CaptureEdge capture_mode;
    unsigned int word_size;    // bits (not including a potential parity bit)
    unsigned int clock_speed;  // in Hertz
    unsigned int transfer_delay;   // in microseconds
  };

  SPIDevice(const Settings &settings)
  {

  }

};

#endif
