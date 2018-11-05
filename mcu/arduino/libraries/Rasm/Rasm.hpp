/**
 * Defines the RasmMotorSet and RasmEncoderSet classes.
 * This file also defines the Joint enumeration and typedefs the MotorState and
 * Pins enumerations from the DualMC33926MotorDriver class.
 *
 * Summary
 *   structs
 *     MotorPins
 *     RasmMotorSet::DriversToJoints
 *     RasmEncoderSet::AdcPins
 *   enumerations
 *     MotorState
 *     Joint
 *   methods
 *     void RasmMotorSet::set_motor_state(Joint joint, MotorState state)
 *     void RasmMotorSet::set_motor_speed(Joint joint, unsigned int duty_cycle)
 *     unsigned int RasmEncoderSet::get_encoder_output(Joint joint)
 */

#ifndef RASM_RASM_HPP
#define RASM_RASM_HPP

#include <Arduino.h>

#include <StandardCplusplus.h>
#include <vector>
#include <map>

#include <DualMC33926MotorDriver.hpp>
#include <Filter.hpp>

typedef DualMC33926MotorDriver::MotorState MotorState;
typedef DualMC33926MotorDriver::Pins MotorPins;

/**
 * Represents the six motorized joints of the RASM.
 */
enum Joint
{
  BASE,
  SHOULDER,
  ELBOW,
  WRIST_YAW,
  WRIST_PITCH,
  WRIST_ROLL
};

/**
 * This class is for controlling all rasm motors via three dual motor drivers.
 */
class RasmMotorSet
{
public:
  struct DriversToJoints
  {
    Joint driver1_m1_joint;
    Joint driver1_m2_joint;
    Joint driver2_m1_joint;
    Joint driver2_m2_joint;
    Joint driver3_m1_joint;
    Joint driver3_m2_joint;
  };

private:
  DualMC33926MotorDriver *dual_driver_1;  // base & shoulder
  DualMC33926MotorDriver *dual_driver_2;  // elbow & wrist-yaw
  DualMC33926MotorDriver *dual_driver_3;  // wrist-pitch & wrist-roll
  DriversToJoints dj_map;

public:
  RasmMotorSet(MotorPins &driver1_pins, MotorPins &driver2_pins,
  MotorPins &driver3_pins, DriversToJoints &dtj)
  {
    dual_driver_1 = new DualMC33926MotorDriver(driver1_pins);
    dual_driver_2 = new DualMC33926MotorDriver(driver2_pins);
    dual_driver_3 = new DualMC33926MotorDriver(driver3_pins);
    dj_map = dtj;
  }

  /**
   * Sets the operating mode of the motor for the given joint to one of the
   * states in MotorState.
   */
  void set_motor_state(Joint joint, MotorState state)
  {
    if (joint == dj_map.driver1_m1_joint)
      dual_driver_1->set_left_motor_state(state);
    else if (joint == dj_map.driver1_m2_joint)
      dual_driver_1->set_right_motor_state(state);
    else if (joint == dj_map.driver2_m1_joint)
      dual_driver_2->set_left_motor_state(state);
    else if (joint == dj_map.driver2_m2_joint)
      dual_driver_2->set_right_motor_state(state);
    else if (joint == dj_map.driver3_m1_joint)
      dual_driver_3->set_left_motor_state(state);
    else if (joint == dj_map.driver3_m2_joint)
      dual_driver_3->set_right_motor_state(state);
  }

  /**
   * Sets the magnitude of the motor speed for the given joint to an integer
   * percentage from 0 to 100 which is given by duty_cycle.
   */
  void set_motor_speed(Joint joint, unsigned int duty_cycle)
  {
    if (joint == dj_map.driver1_m1_joint)
      dual_driver_1->set_left_motor_speed(duty_cycle);
    else if (joint == dj_map.driver1_m2_joint)
      dual_driver_1->set_right_motor_speed(duty_cycle);
    else if (joint == dj_map.driver2_m1_joint)
      dual_driver_2->set_left_motor_speed(duty_cycle);
    else if (joint == dj_map.driver2_m2_joint)
      dual_driver_2->set_right_motor_speed(duty_cycle);
    else if (joint == dj_map.driver3_m1_joint)
      dual_driver_3->set_left_motor_speed(duty_cycle);
    else if (joint == dj_map.driver3_m2_joint)
      dual_driver_3->set_right_motor_speed(duty_cycle);
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
    std::vector<double> ff_coeffs;
    std::vector<double> fb_coeffs;
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
    for (Joint j = 0; j < 6; j = j+1)
      filter_map[j] = new CausalLTIFilter(coeffs.ff_coeffs, coeffs.fb_coeffs);
  }

  /**
   * Takes a new encoder reading for the given joint, filters it, and returns
   * the new filter output.
   */
  unsigned int get_encoder_output(Joint joint)
  {
    filter_map[joint]->input(analogRead(pin_map[joint]));
    return filter_map[joint]->output();
  }
};

#endif
