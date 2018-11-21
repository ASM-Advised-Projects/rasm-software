/**
 * This program runs a command interface over Serial for controlling the RASM's
 * six motors and reading the six encoders. The serial communication protocol
 * for the three supported commands is defined below. Note that 'receive' and
 * 'transmit' can also be view as 'request' and 'response'.
 *
 * Ping
 *     receive: p
 *       p - header char
 *     transmit: a
 *       a - acknowledgement char
 * Set Motor
 *     receive: m[0-5][frz][0-1][0-9][0-9]
 *       m - header char
 *       [0-5] - joint select
 *       [frz] - motor state (forward, reverse, or high-z)
 *       [0-1][0-9][0-9] - big-endian motor duty cycle in range [0,100]
 *     transmit: a
 *       a - acknowledgement char
 * Read Encoder
 *     receive: e[0-5]
 *       e - header char
 *       [0-5] - joint select
 *     transmit: [0-9][0-9][0-9][0-1]
 *       [0-9][0-9][0-9][0-1] - little-endian encoder value in range [0,1023]
 *
 * The serial read timeout is set to 50 milliseconds. If a timeout ever occurs
 * when waiting for the rest of a received command after the initial 'header
 * char' has been read, then only an 'n' char will be transmitted. By the way,
 * the 'a' char stands for 'acknowledgement' while the 'n' char stands for
 * 'negative acknowledgement'.
 */

#include <Rasm.hpp>

RasmMotorSet *motors;
RasmEncoderSet *encoders;

Joint joint;
int level;

void set_pwm_frequency()
{
  // set divisors of 8 for timers 3 and 4
  // this will provide a frequency of 16MHz / 256 / 8 = 7812.5 Hz
  TCCR3B = (TCCR3B & 0b11111000) | 0x02;
  TCCR4B = (TCCR4B & 0b11111000) | 0x02;

  //struct Timer {T0, T1, T2, T3, T4, T5};
  //struct Divisor {_1, _8, _64, _256, _1024};
}

void setup()
{
  // initialize serial port
  Serial.begin(9600);
  Serial.setTimeout(100);

  /*
   * Set frequencies for pwm pins (MotorPins::mx_IN2) used by motor drivers.
   * The default frequency for pwm (analogWrite) output is <1kHz unless the
   * prescalar/divisor for the timer backing that pwm pin is modified.
   *
   * Note that Timer0 is used by the arduino core so no-touchy.
   * Here's a timer-to-pin map for the Mega 2650 (ATmega1280):
   * timer   pin1  pin2  pin3
   * ------  ----  ----  ----
   * Timer0   4     13    --
   * Timer1   11    12    --
   * Timer2   9     10    --
   * Timer3   2     3     5
   * Timer4   6     7     8
   * Timer5   44    45    46
   *
   * The millis() and delay() functions rely on some combination of timers 0, 1,
   * and 2, so timers 3 and 4 will have their frequencies set to be used by the
   * motor drivers. This will allow for 6 total pwm outputs on pins 2,3,5,6,7,8
   * that are of the desired frequency.
   *
   * The frequency should be greater than 1k (will likely reduce how obnoxious
   * the humming sound of a slow-moving motor is) but less than 11k (so that
   * the motor drivers don't need to be put into a high slew-rate mode).
   * This function changes the frequency of timers 3 and 4 to 7812.5 Hz.
   */
  set_pwm_frequency();

  // initialize motors
  //   - All pins are digital-output except for the m1_IN2 and m2_IN2 pins which
  //     are analog output and hence must be pwm capable.
  //   - Right now all digital pins are 22 but should be changed to something
  //     between 22 and 53 and should all be unique.

  // pin assignments for dual motor driver #1
  // elbow (m1) and shoulder (m2)
  MotorPins motor_pins1;
  motor_pins1.m1_IN1 = 23;
  motor_pins1.m1_IN2 = 2;
  motor_pins1.m1_D1 = 22;
  motor_pins1.m1_D2 = 22;
  motor_pins1.m2_IN1 = 24;
  motor_pins1.m2_IN2 = 3;
  motor_pins1.m2_D1 = 22;
  motor_pins1.m2_D2 = 22;
  motor_pins1.enable = 22;
  motor_pins1.slew = 22;

  // pin assignments for dual motor driver #2
  MotorPins motor_pins2;
  motor_pins2.m1_IN1 = 25;
  motor_pins2.m1_IN2 = 5;
  motor_pins2.m1_D1 = 22;
  motor_pins2.m1_D2 = 22;
  motor_pins2.m2_IN1 = 26;
  motor_pins2.m2_IN2 = 6;
  motor_pins2.m2_D1 = 22;
  motor_pins2.m2_D2 = 22;
  motor_pins2.enable = 22;
  motor_pins2.slew = 22;

  // pin assignments for dual motor driver #3
  MotorPins motor_pins3;
  motor_pins3.m1_IN1 = 27;
  motor_pins3.m1_IN2 = 7;
  motor_pins3.m1_D1 = 22;
  motor_pins3.m1_D2 = 22;
  motor_pins3.m2_IN1 = 28;
  motor_pins3.m2_IN2 = 8;
  motor_pins3.m2_D1 = 22;
  motor_pins3.m2_D2 = 22;
  motor_pins3.enable = 22;
  motor_pins3.slew = 22;

  // driver-to-joint assignments
  RasmMotorSet::DriversToJoints dtj;
  dtj.driver1_m1_joint = Joint::ELBOW;
  dtj.driver1_m2_joint = Joint::SHOULDER;
  dtj.driver2_m1_joint = Joint::WRIST_ROLL;
  dtj.driver2_m2_joint = Joint::BASE;
  dtj.driver3_m1_joint = Joint::WRIST_YAW;
  dtj.driver3_m2_joint = Joint::WRIST_PITCH;

  motors = new RasmMotorSet(motor_pins1, motor_pins2, motor_pins3, dtj);
  for (Joint joint = 0; joint < 6; joint = joint + 1)
  {
    motors->set_motor_state(joint, MotorState::FORWARD);
    motors->set_motor_speed(joint, 0);
  }

  // initialize encoders
  // analog-read pin assignments for each joint
  RasmEncoderSet::AdcPins encoder_pins;
  encoder_pins.base = A0;
  encoder_pins.shoulder = A1;
  encoder_pins.elbow = A2;
  encoder_pins.wristyaw = A5;
  encoder_pins.wristpitch = A5;
  encoder_pins.wristroll = A5;

  // filter coefficients that are applied to each encoder
  RasmEncoderSet::FilterCoeffs encoder_coeffs;
  encoder_coeffs.ff_coeffs.push_back(1);
  encoder_coeffs.fb_coeffs.push_back(0);

  encoders = new RasmEncoderSet(encoder_pins, encoder_coeffs);

  joint = Joint::BASE;
  level = 0;
}

void loop()
{
  if (Serial.available())
  {
    String s = Serial.readString();
    char c = s.charAt(0);
    if (c == 'j')  // change joint
    {
      char j = s.charAt(1) - '0';
      joint = (Joint)(j);
    }
    else if (c == 'f')
    {
      motors->set_motor_state(joint, MotorState::FORWARD);
    }
    else if (c == 'r')
    {
      motors->set_motor_state(joint, MotorState::REVERSE);
    }
    else
    {
      Serial.println();
      Serial.println();
      level = c - '0';
      if (level >= 0 && level <= 9)
      {
        motors->set_motor_speed(joint, 10*level);
      }
    }
  }

  if (level != 0)
  {
    Serial.print(millis());
    Serial.print("\t");
    Serial.println(encoders->get_encoder_output(joint));
  }

  return;

  // continue only if serial data is available
  if (!Serial.available())
    return;

  // pre-declarations to avoid scoping problems in the switch block
  char buf[5];
  int len;
  Joint joint;
  MotorState state;
  unsigned int dutycycle;
  int encVal;

  char headerChar = Serial.read();
  switch (headerChar)
  {
    // ping command
    // receive: p
    // transmit: a
    case 'p':
      Serial.write('a');
      break;

    // set motor speed command
    // receive: m[0-5][frz][0-9][0-9][0-9] (big-endian)
    // transmit (success): a
    // transmit (failure): n
    case 'm':
      len = Serial.readBytes(buf, 5);
      if (len < 5)
      {
        Serial.write('n');
      }
      else
      {
        joint = (Joint)(buf[0]-'0');
        state = char_to_motor_state(buf[1]);
        dutycycle = 0;
        for (int i = 0; i < 3; i++)
          dutycycle = 10*dutycycle + (buf[2+i]-'0');
        motors->set_motor_state(joint, state);
        motors->set_motor_speed(joint, dutycycle);
        Serial.write('a');
      }
      break;

    // read encoder command
    // receive: e[0-5]
    // transmit (success): [0-9][0-9][0-9][0-9] (little-endian)
    // transmit (failure): n
    case 'e':
      len = Serial.readBytes(buf, 1);
      if (len < 1)
      {
        Serial.write('n');
      }
      else
      {
        joint = (Joint)(buf[0]-'0');
        encVal = encoders->get_encoder_output(joint);
        for (int i = 0; i < 4; i++)
        {
          Serial.write((char)((encVal%10) + '0'));
          encVal /= 10;
        }
      }
      break;

    // invalid header char
    // transmit: n
    default:
      Serial.write('n');
      break;
  }
  Serial.flush();
}

MotorState char_to_motor_state(char c)
{
  switch (c)
  {
    case 'f':
      return MotorState::FORWARD;
    case 'r':
      return MotorState::REVERSE;
    case 'z':
      return MotorState::HIGHZ;
    default:
      return MotorState::HIGHZ;
  }
}
