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

#define SPEED_SETPOINT		0.0
#define SPEED_KP 			0.0
#define SPEED_KI 			0.0
#define SPEED_KD 			0.0
#define ANGLE_SETPOINT 		0.0
#define ANGLE_LIMIT 		0.0
#define ANGLE_KP_AGGR 		0.0
#define ANGLE_KI_AGGR 		0.0
#define ANGLE_KD_AGGR 		0.0
#define ANGLE_KP_CONS 		2.0
#define ANGLE_KI_CONS 		0.0
#define ANGLE_KD_CONS 		0.0
#define ANGLE_IRRECOVERABLE 45.0
#define CALIBRATED_ZERO_ANGLE 0.0
#define WINDUP_GUARD 		100

enum PIDTuning {CONSERVATIVE, AGGRESSIVE};

class PID
{
public:
	PID();
	float compute (float input);
    void setSetpoint(float setpoint);
    void setTunings(float Kp, float Ki, float Kd);
    void getParameters(float *setpoint, float *Kp, float *Ki, float *Kd);
private:
	float lastError;
	unsigned long lastTime;
    float setpoint;
    float Cp;
    float Ci;
    float Cd;
    float Kp;
    float Ki;
    float Kd;
};

#endif
