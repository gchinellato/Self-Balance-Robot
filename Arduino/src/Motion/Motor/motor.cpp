/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Motor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/

#include <Arduino.h>
#include "motor.h"

Motor::Motor()
{
    /*  VNH2SP30 pin definitions
     xxx[0] controls '1' outputs
     xxx[1] controls '2' outputs */
    inApin[0] = 9;  // INA: Clockwise input
    inApin[1] = 10;  // INA: Clockwise input

    inBpin[0] = 11; // INB: Counter-clockwise input
    inBpin[1] = 12; // INB: Counter-clockwise input

    pwmpin[0] = 5; // PWM input
    pwmpin[1] = 6; // PWM input

    cspin[0] = 2; // CS: Current sense ANALOG input
    cspin[1] = 3; // CS: Current sense ANALOG input
    statpin = 13;
}

void Motor::Initialize()
{
    // Initialize digital pins as outputs
    for (int i=0; i<2; i++)
    {
      pinMode(inApin[i], OUTPUT);
      pinMode(inBpin[i], OUTPUT);
      pinMode(pwmpin[i], OUTPUT);
    }
    // Initialize braked
    for (int i=0; i<2; i++)
    {
      digitalWrite(inApin[i], LOW);
      digitalWrite(inBpin[i], LOW);
    }
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.

 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled

 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND

 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void Motor::motorGo(int motor, int direct, int pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void Motor::motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

void Motor::CheckCurrentSense()
{
    if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD))
        digitalWrite(statpin, HIGH);
}
