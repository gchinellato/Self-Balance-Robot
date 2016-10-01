/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: PID
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/


#ifndef PID_H
#define PID_H

#define WINDUP_GUARD 255

class PID
{
public:
	PID();
	float compute (float input, float dt);
    void setSetpoint(float setpoint);
    void setTunings(float Kp, float Ki, float Kd);
    void getParameters(float *setpoint, float *Kp, float *Ki, float *Kd);
private:
	float lastError;
    float setpoint;
    float Kp;
    float Ki;
    float Kd;
};

#endif
