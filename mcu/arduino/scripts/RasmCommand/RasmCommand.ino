/**
 * This program runs a command interface over Serial for controlling the RASM's
 * six motors and reading the six encoders.
 *
 * Serial communication protocol:
 *   ping
 *     receive: p
 *     transmit: a
 *   set motor
 *     receive: m[0-5][frz][0-9][0-9][0-9] (big-endian)
 *     transmit: a
 *   set encoder
 *     receive: e[0-5]
 *     transmit: [0-9][0-9][0-9][0-9] (little-endian)
 *
 * If a timeout (set to 50 milliseconds) ever occurs when waiting for the rest
 * of a received command then an 'n' char is transmitted instead. Note that the
 * 'a' char stands for ACK (acknowledgement) while the 'n' char stands for NAK
 * (negative acknowledgement).
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
  MotorPins motor_pins1; // for dual motor driver 1
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

  MotorPins motor_pins2;  // for dual motor driver 2
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

  MotorPins motor_pins3;  // for dual motor driver 3
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

  RasmMotorSet::DriversToJoints dtj;
  dtj.driver1_m1_joint = Joint::SHOULDER;
  dtj.driver1_m2_joint = Joint::ELBOW;
  dtj.driver2_m1_joint = Joint::BASE;
  dtj.driver2_m2_joint = Joint::WRIST_PITCH;
  dtj.driver3_m1_joint = Joint::WRIST_YAW;
  dtj.driver3_m2_joint = Joint::WRIST_ROLL;

  motors = new RasmMotorSet(motor_pins1, motor_pins2, motor_pins3, dtj);

  // initialize encoders
  RasmEncoderSet::AdcPins encoder_pins;
  encoder_pins.base = A0;
  encoder_pins.shoulder = A1;
  encoder_pins.elbow = A2;
  encoder_pins.wristyaw = A3;
  encoder_pins.wristpitch = A4;
  encoder_pins.wristroll = A5;

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
