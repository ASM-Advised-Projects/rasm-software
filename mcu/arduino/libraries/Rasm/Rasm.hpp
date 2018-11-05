/**
 * Defines the Joint enumeration along with the RasmMotorSet and RasmEncoderSet
 * classes.
 */

#ifndef RASM_RASM_HPP
#define RASM_RASM_HPP

#include <Arduino.h>

#include <StandardCplusplus.h>
#include <vector>
#include <valarray>
#include <map>

#include <DualMC33926MotorDriver.hpp>
#include <Filter.hpp>

using std::vector;
using std::valarray;

typedef MotorState DualMC33926MotorDriver::MotorState;

/**
 * Represents the six motorized joints of the RASM.
 */
enum Joint
{
  BASE = 0,
  SHOULDER = 1,
  ELBOW = 2,
  WRIST_YAW = 3,
  WRIST_PITCH = 4,
  WRIST_ROLL = 5
};

/**
 * This class is for controlling all rasm motor drivers.
 */
class RasmMotorSet
{
private:
  DualMC33926MotorDriver *dual_driver_1;  // base & shoulder
  DualMC33926MotorDriver *dual_driver_2;  // elbow & wrist-yaw
  DualMC33926MotorDriver *dual_driver_3;  // wrist-pitch & wrist-roll

public:
  RasmMotorSet(DualMC33926MotorDriver::Pins driver1_pins)
  {
    dual_driver_1 = new DualMC33926MotorDriver(driver1_pins);
  }

  void set_motor_state(Joint joint, MotorState state)
  {
    switch (joint)
    {
      case BASE:
        return;
        break;

      case SHOULDER:
        dual_driver_1->set_right_motor_state(state);
        break;

      case ELBOW:
        return;
        break;

      case WRIST_YAW:
        return;
        break;

      case WRIST_PITCH:
        return;
        break;

      case WRIST_ROLL:
        return;
        break;
    }
  }

  void set_motor_speed(Joint joint, unsigned int duty_cycle)
  {
    switch (joint)
    {
      case BASE:
        return;
        break;

      case SHOULDER:
        dual_driver_1->set_right_motor_speed(duty_cycle);
        break;

      case ELBOW:
        return;
        break;

      case WRIST_YAW:
        return;
        break;

      case WRIST_PITCH:
        return;
        break;

      case WRIST_ROLL:
        return;
        break;
    }
  }
};

/**
 * This class is for reading and filtering all rasm encoders.
 */
class RasmEncoderSet
{
private:
  std::map<Joint, unsigned int> pin_map;
  std::map<Joint, CausalLTIFilter*> filter_map;

public:
  struct AdcPins {
    unsigned int base;
    unsigned int shoulder;
    unsigned int elbow;
    unsigned int wristyaw;
    unsigned int wristpitch;
    unsigned int wristroll;
  };

  struct FilterCoeffs {
    vector<double> ff_coeffs;
    vector<double> fb_coeffs;
  };

  RasmEncoderSet(AdcPins &pins, FilterCoeffs &coeffs)
  {
    // set pin modes of each adc pin
    pinMode(pins.base, INPUT);
    pinMode(pins.shoulder, INPUT);
    pinMode(pins.elbow, INPUT);
    pinMode(pins.wristyaw, INPUT);
    pinMode(pins.wristpitch, INPUT);
    pinMode(pins.wristroll, INPUT);

    // initialize joint-to-adc pin number map
    pin_map[Joint::BASE] = pins.base;
    pin_map[Joint::SHOULDER] = pins.shoulder;
    pin_map[Joint::ELBOW] = pins.elbow;
    pin_map[Joint::WRIST_YAW] = pins.wristyaw;
    pin_map[Joint::WRIST_PITCH] = pins.wristpitch;
    pin_map[Joint::WRIST_ROLL] = pins.wristroll;

    // initialize joint-to-filter map
    for (Joint j = BASE; j < 6; j = j+1)
      filter_map[j] = new CausalLTIFilter(coeffs.ff_coeffs, coeffs.fb_coeffs);
  }

  /**
   * Takes a new encoder reading for the given joint, filters it, and returns
   * the new filter output.
   */
  int get_encoder_output(Joint joint)
  {
    filter_map[joint]->input(analogRead(pin_map[joint]));
    return filter_map[joint]->output();
  }

  /**
   * reads all encoders, filters each of the readings, and returns the new
   * output of the filters as an array with the following order of represenation:
   *   base, shoulder, elbow, wrist-yaw, wrist-pitch, wrist-roll
   */
  valarray<int> get_encoder_outputs()
  {
    valarray<int> outputs(6);
    for (Joint j = BASE; j < 6; j = j+1)
      outputs[(int)j] = get_encoder_output(j);
    return outputs;
  }
};

#endif
