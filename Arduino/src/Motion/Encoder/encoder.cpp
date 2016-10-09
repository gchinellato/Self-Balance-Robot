/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Encoder
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#include <Arduino.h>
#include "encoder.h"

Encoder::Encoder(int pin1, int pin2)
{
    this->pin1 = pin1;
    this->pin2 = pin2;
    cmPerTick = 0;
    ticks = 0;

    setTicksPerTurn(TICKS_PER_TURN);
    setRadius(WHEEL_RADIUS);
    setCmPerTick();
}

int Encoder::getTicksPerTurn()
{
    return ticksPerTurn;
}

float Encoder::getRadius()
{
    return radius;
}

/*Get the distance (cm) from the stacionary position, checking the current ticks multiplied by one complete turn in tickes*/
float Encoder::getDistance()
{
    return ticks * cmPerTick;
}

//Set the the lenght of the wheel, C=2*pi*radius/ticksPerTurn
void Encoder::setCmPerTick()
{
    if(ticksPerTurn > 0)
    {
        cmPerTick = (2*M_PI*radius/ticksPerTurn);
    }
}

/*Set radius of the wheels in cm*/
void Encoder::setRadius(float radius)
{
    this->radius = radius;
}

/*Set the number of tick to complete one turn*/
void Encoder::setTicksPerTurn(int ticksPerTurn)
{
    this->ticksPerTurn = ticksPerTurn;
}

void Encoder::resetTicks()
{
    this->ticks = 0;
}
