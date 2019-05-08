

/**
 * This program runs a command interface over Serial1 for interacting with the
 * RASM's electrical and electromechanical systems. More specifically, controlling
 * the six motors, reading the six encoders, reading the battery voltage, and
 * pinging are made possible by this program. The serial communication protocol
 * for these four commands is defined below. Note that in this context, 'receive'
 * and 'transmit' are synonyms for 'request' and 'response'.
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
 *       [frz] - motor state (forward, reverse, or high-impedance)
 *       [0-9][0-9] - big-endian motor duty cycle in range [0,99]
 *     transmit: none
 * Read Encoder
 *     receive: e[0-5]
 *       e - header char
 *       [0-5] - joint select
 *     transmit: [0-9][0-9][0-9][0-1]
 *       [0-9][0-9][0-9][0-1] - little-endian encoder value in range [0,1023]
 * Read Battery
 *     receive: b
 *       b - header char
 *     transmit: [0-9][0-9][0-9][0-1]
 *       [0-9][0-9][0-9][0-1] - little-endian ADC value in range [0,1023]
 *
 * The serial read timeout is set to 50 milliseconds. If a timeout ever occurs
 * when waiting for the rest of a received command after the initial header
 * char has been read, then nothing will be transmitted.
 *
 * If any one of the motors is set to a non-zero level and none of the motors
 * is subsequently set within the next 100 milliseconds then all motor levels
 * will be automatically reset to a speed  of 0. This is a fail-safe meant to
 * keep the joints from going out of range if there is an inadvertent disconnect.
 */

#include <Rasm.hpp>

// function pre-declarations:
void set_pwm_frequency();
void zero_motors();

// settings
static const int serial_timeout = 50;
static const int override_delay = 100;

RasmMotorSet *motors;
RasmEncoderSet *encoders;
int battery_pin;
long last_motor_set_time = 0;
bool motors_zeroed = false;


void setup()
{
  Serial.begin(9600);
  // initialize serial port
  Serial.setTimeout(serial_timeout);

  // configure pwm frequencies for certain motor driver pins
  set_pwm_frequency();

  // initialize motor control pins
  // All pins are digital-output except for the m1_IN2 and m2_IN2 pins
  // which are analog output and therefore must be pwm capable.

  // pin assignments for dual motor driver #1
  // shoulder (m1) and elbow (m2)
  DualMotorPins dmp1;
  dmp1.m1_IN1 = 28;
  dmp1.m1_IN2 = 8;
  dmp1.m1_D1 = 22;
  dmp1.m1_D2 = 22;
  dmp1.m2_IN1 = 27;
  dmp1.m2_IN2 = 7;
  dmp1.m2_D1 = 22;
  dmp1.m2_D2 = 22;
  dmp1.enable = 22;
  dmp1.slew = 22;

  // pin assignments for dual motor driver #2
  // m1 is wrist-pitch and m2 is wrist roll
  DualMotorPins dmp2;
  dmp2.m1_IN1 = 26;
  dmp2.m1_IN2 = 6;
  dmp2.m1_D1 = 22;
  dmp2.m1_D2 = 22;
  dmp2.m2_IN1 = 25;
  dmp2.m2_IN2 = 5;
  dmp2.m2_D1 = 22;
  dmp2.m2_D2 = 22;
  dmp2.enable = 22;
  dmp2.slew = 22;

  // pin assignments for dual motor driver #3
  // m1 is wrist-roll and m2 is base
  DualMotorPins dmp3;
  dmp3.m1_IN1 = 24;
  dmp3.m1_IN2 = 3;
  dmp3.m1_D1 = 22;
  dmp3.m1_D2 = 22;
  dmp3.m2_IN1 = 23;
  dmp3.m2_IN2 = 2;
  dmp3.m2_D1 = 22;
  dmp3.m2_D2 = 22;
  dmp3.enable = 22;
  dmp3.slew = 22;

  // driver-to-joint assignments
  RasmMotorSet::DriversToJoints dtj;
  dtj.driver1_m1_joint = Joint::SHOULDER;
  dtj.driver1_m2_joint = Joint::ELBOW;
  dtj.driver2_m1_joint = Joint::WRIST_PITCH;
  dtj.driver2_m2_joint = Joint::WRIST_ROLL;
  dtj.driver3_m1_joint = Joint::WRIST_YAW;
  dtj.driver3_m2_joint = Joint::BASE;

  motors = new RasmMotorSet(dmp1, dmp2, dmp3, dtj);
  zero_motors();

  // initialize encoders
  // analog-read pin assignments for each joint
  RasmEncoderSet::AdcPins encoder_pins;
  encoder_pins.base = A5;
  encoder_pins.shoulder = A4;
  encoder_pins.elbow = A3;
  encoder_pins.wristyaw = A2;
  encoder_pins.wristpitch = A1;
  encoder_pins.wristroll = A0;

  encoders = new RasmEncoderSet(encoder_pins);

  // initialize battery-read pin
  battery_pin = A6;
}


void loop()
{
  // override motor speeds to 0 if too much time has
  // passed since the last motor set command
  if (!motors_zeroed && millis() - last_motor_set_time > override_delay)
    zero_motors();

  // continue only if serial data is available
  if (!Serial.available())
    return;

  //Serial.println("received");

  // pre-declarations to avoid scoping problems accross switch cases
  char buf[5];
  Joint joint;
  int dutycycle, enc_val, batt_val;

  char header_char = Serial.read();
  switch (header_char)
  {
    // ping
    // receive: p
    // transmit: a
    case 'p':
      Serial.write('a');
      Serial.flush();
      break;

    // set motor speed
    // receive: m[0-5][frz][0-9][0-9] (big-endian)
    // transmit: none
    case 'm':
      if (Serial.readBytes(buf, 4) < 4)
        break;

      joint = (Joint)(buf[0]-'0');
      MotorState state;
      if (buf[1] == 'f')
        state = MotorState::FORWARD;
      else if (buf[1] == 'r')
        state = MotorState::REVERSE;
      else
        state = MotorState::HIGHZ;
      dutycycle = 10*(buf[2]-'0') + (buf[3]-'0');
      motors->set_motor_state(joint, state);
      motors->set_motor_speed(joint, dutycycle);

      last_motor_set_time = millis();
      motors_zeroed = false;
      break;

    // read encoder
    // receive: e[0-5]
    // transmit: [0-9][0-9][0-9][0-9] (little-endian)
    case 'e':
      if (Serial.readBytes(buf, 1) < 1)
        break;

      joint = (Joint)(buf[0]-'0');
      enc_val = encoders->get_encoder_output(joint);
      for (int i = 0; i < 4; i++)
      {
        Serial.write((char)((enc_val%10) + '0'));
        enc_val /= 10;
      }
      Serial.flush();
      break;

    // read battery
    case 'b':
      batt_val = analogRead(battery_pin);
      for (int i = 0; i < 4; i++)
      {
        Serial.write((char)((batt_val%10) + '0'));
        batt_val /= 10;
      }
      Serial.flush();
      break;

    default:
      return;
  }
}


/**
 * This function changes the frequency of timers 3 and 4 (pins 2,3,5,6,7,8) to
 * 7812.5 Hz.
 *
 * The default frequency for pwm (analogWrite) output is <1kHz unless the
 * prescalar/divisor for the timer backing that pwm pin is modified. The
 * frequency should be greater than 1k (will likely reduce how obnoxious the
 * humming sound of a slow-moving motor is) but less than 11k (so that the motor
 * drivers don't need to be put into a high slew-rate mode).
 *
 * Note that Timer0 is used by the arduino core so no-touchy.
 * Here's a timer-to-pin map for the Mega 2560 (uses the ATmega2560):
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
 */
void set_pwm_frequency()
{
  // set divisors of 8 for timers 3 and 4
  // this will provide a frequency of 16MHz / 256 / 8 = 7812.5 Hz
  TCCR3B = (TCCR3B & 0b11111000) | 0x02;
  TCCR4B = (TCCR4B & 0b11111000) | 0x02;

  //struct Timer {T0, T1, T2, T3, T4, T5};
  //struct Divisor {_1, _8, _64, _256, _1024};
}


/**
 * Sets all motor states to forward and motor speeds to 0.
 * Also, sets motors_zeroed to true.
 */
void zero_motors()
{
  for (Joint joint = 0; joint < 6; joint = joint + 1)
  {
    motors->set_motor_state(joint, MotorState::FORWARD);
    motors->set_motor_speed(joint, 0);
  }
  motors_zeroed = true;
}
