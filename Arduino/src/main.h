/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Main thread
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/

#ifndef MAIN_H
#define MAIN_H

#define SERIAL_BAUDRATE 	115200
#define CALIBRATION_MAGNETO	0
#define DATA_INTERVAL 	    20 // ms

float dt=0; // gyroscope integration interval
unsigned int timestamp;
unsigned int timestamp_old;
float *ori;

struct Configuration {
	double speedPIDKp;
	double speedPIDKi;
	double speedPIDKd;
	double speedPIDOutputLowerLimit;
	double speedPIDOutputHigherLimit;
	double anglePIDAggKp;
	double anglePIDAggKi;
	double anglePIDAggKd;
	double anglePIDConKp;
	double anglePIDConKi;
	double anglePIDConKd;
	double anglePIDLowerLimit;
	double calibratedZeroAngle;
	int motor1MinimumSpeed;
	int motor2MinimumSpeed;
};

struct UserControl {
	float steering;
	float direction;
};

void setConfiguration();

#endif
