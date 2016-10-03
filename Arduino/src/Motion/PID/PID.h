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
#define ANGLE_LIMIT 		5.0
#define ANGLE_KP_AGGR 		10.0
#define ANGLE_KI_AGGR 		0.0
#define ANGLE_KD_AGGR 		0.0
#define ANGLE_KP_CONS 		6.0
#define ANGLE_KI_CONS 		0.7
#define ANGLE_KD_CONS 		0.2
#define ANGLE_IRRECOVERABLE 30.0
#define CALIBRATED_ZERO_ANGLE -2.0
#define WINDUP_GUARD 		255

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
	float lastTime;
    float setpoint;
    float Kp;
    float Ki;
    float Kd;
};

#endif
