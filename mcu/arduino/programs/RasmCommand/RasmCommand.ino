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

void setup()
{
  // initialize serial port
  Serial.begin(115200);
  Serial.setTimeout(50);

  // initialize motors
  // pin assignments for dual motor driver #1
  MotorPins motor_pins1;
  motor_pins1.m1_IN1 = 10;
  motor_pins1.m1_IN2 = 10;
  motor_pins1.m1_D1 = 10;
  motor_pins1.m1_D2 = 10;
  motor_pins1.m2_IN1 = 2;
  motor_pins1.m2_IN2 = 3;
  motor_pins1.m2_D1 = 10;
  motor_pins1.m2_D2 = 10;
  motor_pins1.enable = 10;
  motor_pins1.slew = 10;

  // pin assignments for dual motor driver #2
  MotorPins motor_pins2;
  motor_pins2.m1_IN1 = 10;
  motor_pins2.m1_IN2 = 10;
  motor_pins2.m1_D1 = 10;
  motor_pins2.m1_D2 = 10;
  motor_pins2.m2_IN1 = 10;
  motor_pins2.m2_IN2 = 10;
  motor_pins2.m2_D1 = 10;
  motor_pins2.m2_D2 = 10;
  motor_pins2.enable = 10;
  motor_pins2.slew = 10;

  // pin assignments for dual motor driver #3
  MotorPins motor_pins3;
  motor_pins3.m1_IN1 = 10;
  motor_pins3.m1_IN2 = 10;
  motor_pins3.m1_D1 = 10;
  motor_pins3.m1_D2 = 10;
  motor_pins3.m2_IN1 = 10;
  motor_pins3.m2_IN2 = 10;
  motor_pins3.m2_D1 = 10;
  motor_pins3.m2_D2 = 10;
  motor_pins3.enable = 10;
  motor_pins3.slew = 10;

  // driver-to-joint assignments
  RasmMotorSet::DriversToJoints dtj;
  dtj.driver1_m1_joint = Joint::SHOULDER;
  dtj.driver1_m2_joint = Joint::ELBOW;
  dtj.driver2_m1_joint = Joint::BASE;
  dtj.driver2_m2_joint = Joint::WRIST_PITCH;
  dtj.driver3_m1_joint = Joint::WRIST_YAW;
  dtj.driver3_m2_joint = Joint::WRIST_ROLL;

  motors = new RasmMotorSet(motor_pins1, motor_pins2, motor_pins3, dtj);

  // initialize encoders
  // analog-read pin assignments for each joint
  RasmEncoderSet::AdcPins encoder_pins;
  encoder_pins.base = A0;
  encoder_pins.shoulder = A1;
  encoder_pins.elbow = A2;
  encoder_pins.wristyaw = A3;
  encoder_pins.wristpitch = A4;
  encoder_pins.wristroll = A5;

  // filter coefficients that are applied to each encoder
  RasmEncoderSet::FilterCoeffs encoder_coeffs;
  encoder_coeffs.ff_coeffs.push_back(1);
  encoder_coeffs.fb_coeffs.push_back(0);

  encoders = new RasmEncoderSet(encoder_pins, encoder_coeffs);
}

void loop()
{
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
