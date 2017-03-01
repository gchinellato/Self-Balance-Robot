/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Motor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/


#ifndef MOTOR_H
#define MOTOR_H

#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

#define PWM_MIN 0
#define PWM_MAX 799

class Motor
{
public:
	Motor(int pinPWM, int pinCW, int pinCCW, int pinCS);
	void setSpeedPercentage(float speed);
    void motorOff();
    void currentSense();
	float motorSpeed;
private:
	void motorGo(int direct, int pwm);
    int inApin;
    int inBpin;
    int pwmpin;
    int cspin;
};

#endif
