/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: PID
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/

#include <Arduino.h>
#include "PID.h"

PID::PID()
{

}

float PID::compute(float input, float dt)
{
    /* Performs a PID computation and returns a control value based on
    the elapsed time (dt) and the error signal from a summing junction
    (the error parameter)*/
    float error;
    float de;
    float Cp;
    float Ci;
    float Cd;

    error = setpoint - input;
    //Calculate delta error
    de = error - lastError;

    //Proportional Term
    Cp = error;

    //Integral Term
    Ci += error*dt;

    //Windup guard for Integral term do not reach very large values
    if(Ci > WINDUP_GUARD)
    {
        Ci = WINDUP_GUARD;
    }
    else if (Ci < - WINDUP_GUARD)
    {
        Ci = -WINDUP_GUARD;
    }

    Cd = 0;
    //to avoid division by zero
    if(dt>0)
    {
        //Derivative term
        Cd = de/dt;
    }

    //Save for the next iteration
    lastError = error;

    //Sum terms: pTerm+iTerm+dTerm
    return Cp*Kp + Ci*Ki + Cd*Kd;
}

void PID::setSetpoint(float value)
{
    setpoint = value;
}

void PID::setTunings(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::getParameters(float *setpoint, float *Kp, float *Ki, float *Kd)
{
    *setpoint = this->setpoint;
    *Kp = this->Kp;
    *Ki = this->Ki;
    *Kd = this->Kd;
}
