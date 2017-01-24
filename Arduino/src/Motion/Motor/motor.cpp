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

Motor::Motor(int pinPWM, int pinCW, int pinCCW, int pinCS)
{
    pwmpin = pinPWM; // PWM input
    inApin = pinCW;  // INA: Clockwise input
    inBpin = pinCCW; // INB: Counter-clockwise input
    cspin = pinCS; // CS: Current sense ANALOG input

    // Initialize digital pins as outputs
    pinMode(inApin, OUTPUT);
    pinMode(inBpin, OUTPUT);

    // Initialize braked
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.

 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND

 pwm: should be a value between ? and 255, higher the number, the faster it'll go
 */
void Motor::motorGo(int direct, int pwm)
{
    if (direct <= BRAKEGND)
    {
      // Set inA
      if (direct <= CW)
        digitalWrite(inApin, HIGH);
      else
        digitalWrite(inApin, LOW);

      // Set inB
      if ((direct==BRAKEVCC)||(direct==CCW))
        digitalWrite(inBpin, HIGH);
      else
        digitalWrite(inBpin, LOW);

      analogWrite(pwmpin, pwm);
    }
}
/* set speed in percentage from -100 to 100 */
void Motor::setSpeedPercentage(float speed)
{
    // anothing above 100 or below -100 is invalid
    if (speed > 100)
        speed = 100;
    else if (speed < -100)
        speed = -100;

    // negative speed
    if (speed > 0) {
        motorGo(CW, (int(255/100.0 * speed)));
    }
    else if (speed < 0){
        motorGo(CCW, (int(-255/100.0 * speed)));
    }
    else {
        motorOff();
    }
}

void Motor::motorOff()
{
    // Initialize braked
    digitalWrite(inApin, LOW);
    digitalWrite(inBpin, LOW);
    analogWrite(pwmpin, 0);
}

void Motor::currentSense()
{
    if ((analogRead(cspin) < CS_THRESHOLD)){}
}
