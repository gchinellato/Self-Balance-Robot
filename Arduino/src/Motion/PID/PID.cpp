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

float PID::compute(float input)
{
    /* Performs a PID computation and returns a control value based on
    the elapsed time (dt) and the error signal from a summing junction
    (the error parameter)*/
    unsigned long now = millis();
    float dt;
    float error;
    float de;
    float Cp;
    float Ci;
    float Cd;
    float output;

    /* Calculate delta time (seconds) */
    dt = (float)(now - lastTime)/1000.0f;
    //Serial.println("dt: " + String(dt));

    /* Calculate delta error */
    error = setpoint - input;
    de = error - lastError;
    //Serial.println("input: " + String(input));
    //Serial.println("de: " + String(de));

    /* Proportional Term */
    Cp = error;
    //Serial.println("cp: " + String(Cp));

    /* Integral Term */
    Ci += error*dt;
    //Serial.println("ci: " + String(Ci));

    Cd = 0;
    /* to avoid division by zero */
    if(dt>0)
    {
        /* Derivative term */
        Cd = de/dt;
        //Serial.println("cd: " + String(Cd));
    }

    /* Save for the next iteration */
    lastError = error;
    lastTime = now;

    /* Sum terms: pTerm+iTerm+dTerm */
    output = Cp*Kp + Ci*Ki + Cd*Kd;
    //Serial.println("output: " + String(output));

    /* Saturation - Windup guard for Integral term do not reach very large values */
    if(output > WINDUP_GUARD){
        output = WINDUP_GUARD;
    }
    else if (output < -WINDUP_GUARD){
        output = -WINDUP_GUARD;
     }

    return output;
}

void PID::setSetpoint(float value)
{
    this->setpoint = value;
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
