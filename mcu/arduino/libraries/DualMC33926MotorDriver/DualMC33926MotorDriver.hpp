/**
 * Defines the DualMC33926MotorDriver class.
 */

#include <Arduino.h>

/**
 * This class is for controlling the Pololu dual MC33926 motor driver board.
 */
class DualMC33926MotorDriver
{
public:
  // forward uses slow decay on pwm low segments via freewheeling high
  // reverse uses slow decay on pwm low segments via freewheeling low
  // brake uses freewheeling
  // forward or reverse at 0 speed is the same as brake
  enum MotorState
  {
    FORWARD,
    REVERSE,
    HIGHZ
  };

  struct Pins {
    unsigned int m1_IN1;
    unsigned int m1_IN2;
    unsigned int m1_D1;
    unsigned int m1_D2;
    unsigned int m2_IN1;
    unsigned int m2_IN2;
    unsigned int m2_D1;
    unsigned int m2_D2;
    unsigned int enable;
    unsigned int slew;
  };

private:
  Pins pins;
  MotorState left_motor_state;
  MotorState right_motor_state;
  unsigned int left_motor_speed;
  unsigned int right_motor_speed;

  void enable_left_motor()
  {
    if (left_motor_state == HIGHZ)
    {
      digitalWrite(pins.m1_D1, LOW);
      digitalWrite(pins.m1_D2, HIGH);
    }
  }

  void enable_right_motor()
  {
    if (right_motor_state == HIGHZ)
    {
      digitalWrite(pins.m2_D1, LOW);
      digitalWrite(pins.m2_D2, HIGH);
    }
  }

public:
  DualMC33926MotorDriver(Pins &pin_assignments)
  : left_motor_state(HIGHZ)
  , right_motor_state(HIGHZ)
  , left_motor_speed(0)
  , right_motor_speed(0)
  {
    pins = pin_assignments;

    // initialize pins
    pinMode(pins.m1_IN1, OUTPUT);
    pinMode(pins.m1_IN2, OUTPUT);
    pinMode(pins.m1_D1, OUTPUT);
    pinMode(pins.m1_D2, OUTPUT);
    pinMode(pins.m2_IN1, OUTPUT);
    pinMode(pins.m2_IN2, OUTPUT);
    pinMode(pins.m2_D1, OUTPUT);
    pinMode(pins.m2_D2, OUTPUT);
    pinMode(pins.enable, OUTPUT);
    pinMode(pins.slew, OUTPUT);

    // set initial motor states to high-impedance
    set_left_motor_state(HIGHZ);
    set_right_motor_state(HIGHZ);
    set_left_motor_speed(0);
    set_right_motor_speed(0);
  }

  void enable()
  {
    digitalWrite(pins.enable, HIGH);
  }

  void disable()
  {
    digitalWrite(pins.enable, LOW);
  }

  void set_hi_slew()
  {
    digitalWrite(pins.slew, HIGH);
  }

  void set_low_slew()
  {
    digitalWrite(pins.slew, HIGH);
  }

  void set_left_motor_state(MotorState state)
  {
    switch (state)
    {
      case FORWARD:
        enable_left_motor();
        digitalWrite(pins.m1_IN1, HIGH);
        analogWrite(pins.m1_IN2, 255-left_motor_speed);
        break;

      case REVERSE:
        enable_left_motor();
        digitalWrite(pins.m1_IN1, LOW);
        analogWrite(pins.m1_IN2, left_motor_speed);
        break;

      case HIGHZ:
        digitalWrite(pins.m1_D1, HIGH);
        digitalWrite(pins.m1_D2, LOW);
        break;
    }
    left_motor_state = state;
  }

  void set_left_motor_speed(unsigned int duty_cycle)
  {
    if (duty_cycle < 0)
      duty_cycle = 0;
    else if (duty_cycle > 100)
      duty_cycle = 100;

    left_motor_speed = map(duty_cycle, 0, 100, 0, 255);
    set_left_motor_state(left_motor_state);
  }

  void set_right_motor_state(MotorState state)
  {
    switch (state)
    {
      case FORWARD:
        enable_right_motor();
        digitalWrite(pins.m1_IN1, HIGH);
        analogWrite(pins.m1_IN2, 255-right_motor_speed);
        break;

      case REVERSE:
        enable_right_motor();
        digitalWrite(pins.m1_IN1, LOW);
        analogWrite(pins.m1_IN2, right_motor_speed);
        break;

      case HIGHZ:
        digitalWrite(pins.m2_D1, HIGH);
        digitalWrite(pins.m2_D2, LOW);
        break;
    }
    right_motor_state = state;
  }

  void set_right_motor_speed(unsigned int duty_cycle)
  {
    if (duty_cycle < 0)
      duty_cycle = 0;
    else if (duty_cycle > 100)
      duty_cycle = 100;

    right_motor_speed = map(duty_cycle, 0, 100, 0, 255);
    set_right_motor_state(right_motor_state);
  }
};
