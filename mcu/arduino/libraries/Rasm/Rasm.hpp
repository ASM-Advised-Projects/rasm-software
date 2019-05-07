/**
 * Defines the RasmMotorSet and RasmEncoderSet classes.
 * This file also defines the Joint enumeration and typedefs the MotorState and
 * Pins enumerations from the DualMC33926MotorDriver class.
 *
 * Summary
 *   structs
 *     DualMotorPins
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




#include <DualMC33926MotorDriver.hpp>

typedef DualMC33926MotorDriver::MotorState MotorState;
typedef DualMC33926MotorDriver::Pins DualMotorPins;

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
  RasmMotorSet(const RasmMotorSet &) = delete;
  RasmMotorSet & operator=(const RasmMotorSet &) = delete;

  RasmMotorSet(DualMotorPins &driver1_pins, DualMotorPins &driver2_pins,
      DualMotorPins &driver3_pins, DriversToJoints &dtj)
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
 * This class is for reading and all rasm encoders.
 */
class RasmEncoderSet
{
private:

    int pin_map[6];


public:
  RasmEncoderSet(const RasmEncoderSet &) = delete;
  RasmEncoderSet & operator=(const RasmEncoderSet &) = delete;

  struct AdcPins {
    unsigned int base;
    unsigned int shoulder;
    unsigned int elbow;
    unsigned int wristyaw;
    unsigned int wristpitch;
    unsigned int wristroll;
  };

  RasmEncoderSet(AdcPins &pins)
  {
    // set pin modes of each adc pin
    pinMode(pins.base, INPUT);
    pinMode(pins.shoulder, INPUT);
    pinMode(pins.elbow, INPUT);
    pinMode(pins.wristyaw, INPUT);
    pinMode(pins.wristpitch, INPUT);
    pinMode(pins.wristroll, INPUT);

    // initialize joint-to-adc pin number map
    pin_map[0] = pins.base;

    pin_map[1] = pins.shoulder;

    pin_map[2] = pins.elbow;

    pin_map[3] = pins.wristyaw;

    pin_map[4] = pins.wristpitch;

    pin_map[5] = pins.wristroll;

  }

  /**
   * Returns a new encoder reading for the given joint.
   */
  unsigned int get_encoder_output(Joint joint)
  {
    return analogRead(pin_map[joint]);
  }
};

#endif
