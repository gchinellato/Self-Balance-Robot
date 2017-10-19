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

#define SERIAL_BAUDRATE 	19200
#define CALIBRATION_MAGNETO	0
#define DATA_INTERVAL 	    10 // ms

/* GPIO mapping */

#define PWM1_PIN 9
#define CW1_PIN 5
#define CCW1_PIN 11
#define CS1_PIN 2

#define PWM2_PIN 10
#define CW2_PIN 6
#define CCW2_PIN 12
#define CS2_PIN 3

#define ENCODERA1_PIN 2
#define ENCODERB1_PIN 4
#define ENCODERA2_PIN 3
#define ENCODERB2_PIN 7

#define TRACE_BEGIN "#BEGIN#"
#define TRACE_END "#END#"

enum cmd{
	STARTED = 0,
	DIRECTION,
	STEERING,
	SPEED_PID,
	ANGLE_PID_AGGR,
	ANGLE_PID_CONS,
	ZERO_ANGLE,
	ANGLE_LIMITE
};

float dt=0; // duration time
unsigned long timestamp;
unsigned long timestamp_old;
float *ori; // orientation vector (roll, pitch, yaw)

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
