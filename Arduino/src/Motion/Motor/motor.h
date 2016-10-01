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

#define CS_THRESHOLD 100

class Motor
{
public:
	Motor();
	void motorGo(int motor, int direct, int pwm);
    void motorOff(int motor);
    void Initialize();
    void CheckCurrentSense();
private:
    int inApin[2];
    int inBpin[2];
    int pwmpin[2];
    int cspin[2];
    int statpin;
};

#endif
