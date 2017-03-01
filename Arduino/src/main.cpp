/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Main thread
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
*/

#include "Arduino.h"
#include <string.h>
#include <Wire.h>
#include "IMU/GY80.h"
#include "Motion/Motor/motor.h"
#include "Motion/Encoder/encoder.h"
#include "Motion/PID/PID.h"
#include "main.h"

Configuration configuration;
UserControl userControl = {0, 0};

// PID variables
float anglePIDSetpoint, anglePIDInput, anglePIDOutput;
float speedPIDInput, speedPIDOutput, speedPIDSetpoint;

float lastDistance1 = 0, lastDistance2 = 0;

// start and stop flag
boolean started = false;

//PID objects
PID speedPID;
PID anglePID;
PIDTuning activePIDTuning = CONSERVATIVE;

//Motors objects
Motor motor1(PWM1_PIN,CW1_PIN,CCW1_PIN,CS1_PIN);
Motor motor2(PWM2_PIN,CW2_PIN,CCW2_PIN,CS2_PIN);

//Encoder objects
Encoder encoder1(ENCODERA1_PIN,ENCODERB1_PIN);
Encoder encoder2(ENCODERA2_PIN,ENCODERB2_PIN);

/* interrupt functions for counting revolutions in the encoders */
/*when the callback function is called due an interrup event on pinEncoder1 and pinEncoder2 is true, then is clockwise, if not it is counter-clockwise*/
void encoderISR1()
{
    if(digitalRead(encoder1.pin2)){ encoder1.ticks++; }
    else{ encoder1.ticks--; }
}

void encoderISR2()
{
    if(digitalRead(encoder2.pin2)){ encoder2.ticks++; }
    else{ encoder2.ticks--; }
}

void setConfiguration()
{
    configuration.speedPIDKp = SPEED_KP;
    configuration.speedPIDKi = SPEED_KI;
    configuration.speedPIDKd = SPEED_KD;
    configuration.speedPIDOutputLowerLimit = -10.00;
    configuration.speedPIDOutputHigherLimit = 10.00;
    configuration.anglePIDAggKp = ANGLE_KP_AGGR;
    configuration.anglePIDAggKi = ANGLE_KP_AGGR;
    configuration.anglePIDAggKd = ANGLE_KP_AGGR;
    configuration.anglePIDConKp = ANGLE_KP_CONS;
    configuration.anglePIDConKi = ANGLE_KP_CONS;
    configuration.anglePIDConKd = ANGLE_KP_CONS;
    configuration.anglePIDLowerLimit = ANGLE_LIMIT;
    configuration.calibratedZeroAngle = CALIBRATED_ZERO_ANGLE;
}

void analogWrite_Init(void)
{
    // Stop the timer while we muck with it
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);

    // Set the timer to mode 14...
    //
    // Mode  WGM13  WGM12  WGM11  WGM10  Timer/Counter Mode of Operation  TOP   Update of OCR1x at TOV1  Flag Set on
    //              CTC1   PWM11  PWM10
    // ----  -----  -----  -----  -----  -------------------------------  ----  -----------------------  -----------
    // 14    1      1      1      0      Fast PWM                         ICR1  BOTTOM                   TOP

    // Set output on Channel A and B to...
    //
    // COM1z1  COM1z0  Description
    // ------  ------  -----------------------------------------------------------
    // 1       0       Clear OC1A/OC1B on Compare Match (Set output to low level).
    TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (0 << WGM10);

    // Set TOP to...
    //
    // fclk_I/O = 16000000
    // N        = 1
    // TOP      = 799
    //
    // fOCnxPWM = fclk_I/O / (N * (1 + TOP))
    // fOCnxPWM = 16000000 / (1 * (1 + 799))
    // fOCnxPWM = 16000000 / 800
    // fOCnxPWM = 20000
    ICR1 = PWM_MAX;

    // Ensure the first slope is complete
    TCNT1 = 0;

    // Ensure Channel A and B start at zero / off
    OCR1A = 0;
    OCR1B = 0;

    // We don't need no stinkin interrupts
    TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

    // Ensure the Channel A and B pins are configured for output
    DDRB |= (1 << DDB1);
    DDRB |= (1 << DDB2);

    // Start the timer...
    //
    // CS12  CS11  CS10  Description
    // ----  ----  ----  ------------------------
    // 0     0     1     clkI/O/1 (No prescaling)
    TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
}

void analogWrite_Timer1(uint8_t pin, int val)
{
    if ((val >= 0) && (val < 800))
    {
        if (pin == 9)
            OCR1A = val;

        if (pin == 10)
            OCR1B = val;
    }
}

void setup()
{
    timestamp=millis();
    Serial.begin(SERIAL_BAUDRATE);
    while(!Serial) {}

    //Caution: beware to change TIMER0 default register.
    //Millis, delay and other functions uses the same TIMER0 as PWM pin 5 and 6.
    //Check https://arduino-info.wikispaces.com/Arduino-PWM-Frequency
    //define PWM pre-scaler to 31372.55 Hz in TIMER1
    //TCCR1B = (TCCR1B & B11111000) | B00000001;

    //define PWM to 20KHz in TIMER1
    //do not use default analogWrite..instead of change OCR1A and OCR1A for D9 and D10 (range 0 up to TOP:ICR1), respectively.
    analogWrite_Init();

    //interrupt pins
    pinMode(encoder1.pin1, INPUT_PULLUP);
    pinMode(encoder2.pin1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder1.pin1), encoderISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2.pin1), encoderISR2, RISING);

    setConfiguration();

/*  GY80 imu;
	if (CALIBRATION_MAGNETO == 1)
	{
		Serial.println("MAGNETOMETER CALIBRATION STARTED");
		imu.magCalibration();
		Serial.println("CALIBRATION FINISHED");
	}*/
}

void loop()
{
    float velocity1=0, velocity2=0;
    float distance1, distance2;
    int command = 0;
    String msg;
    char buff[128];
    char *ret;
    int size = 0;
    int msgLen = 0;

    GY80 imu;

    while(1)
	{
        if ((millis() - timestamp) >= DATA_INTERVAL)
		{
			timestamp_old = timestamp;
			timestamp = millis();

			if (timestamp > timestamp_old)
			{
				dt = (float)(timestamp - timestamp_old) / 1000.0f;
			}
			else
			{
				dt = 0;
			}

            //read serial
            while (Serial.available() > 0){
                msg = Serial.readStringUntil('\n');
                size = msg.length();

                //convert from string to char array
                msg.toCharArray(buff, 128);

                //split string into tokens
                ret = strtok(buff, ",");

                //get message length
                msgLen = atoi(ret);

                //check valid package? received bytes == first parameter?
                if (msgLen == size)
                {
                    //get command
                    command = atoi(strtok(NULL, ","));

                    switch (command) {
                        case STARTED:
                            started = atoi(strtok(NULL, ","));
                            break;
                        case DIRECTION:
                            userControl.direction = atof(strtok(NULL, ","));
                            break;
                        case STEERING:
                            userControl.steering = atof(strtok(NULL, ","));
                            break;
                        case SPEED_PID:
                            configuration.speedPIDKp = atof(strtok(NULL, ","));
                            configuration.speedPIDKi = atof(strtok(NULL, ","));
                            configuration.speedPIDKd = atof(strtok(NULL, ","));
                            speedPID.setTunings(configuration.speedPIDKp, configuration.speedPIDKi, configuration.speedPIDKd);
                            break;
                        case ANGLE_PID_AGGR:
                            configuration.anglePIDAggKp = atof(strtok(NULL, ","));
                            configuration.anglePIDAggKi = atof(strtok(NULL, ","));
                            configuration.anglePIDAggKd = atof(strtok(NULL, ","));
                            break;
                        case ANGLE_PID_CONS:
                            configuration.anglePIDConKp = atof(strtok(NULL, ","));
                            configuration.anglePIDConKi = atof(strtok(NULL, ","));
                            configuration.anglePIDConKd = atof(strtok(NULL, ","));
                            break;
                        case ZERO_ANGLE:
                            configuration.calibratedZeroAngle = atof(strtok(NULL, ","));
                            break;
                        case ANGLE_LIMITE:
                            configuration.anglePIDLowerLimit = atof(strtok(NULL, ","));
                            break;
                        default:
                            //Unknown command
                            break;
                    }
                }
            }

            //read sensors and calculate Euler Angles
			ori = imu.getOrientation(1, dt); //roll pitch yaw
            anglePIDInput = ori[1];

            //update velocity
            //velocity: derivative of position (Pf - Pi)/dt in m/s
            if(dt > 0)
            {
                distance1 = encoder1.getDistance();
                distance2 = encoder2.getDistance();
                //cm/ms -> m/s
                velocity1 = ((distance1 - lastDistance1)*10.0)/dt;
                velocity2 = ((distance2 - lastDistance2)*10.0)/dt;
                lastDistance1 = distance1;
                lastDistance2 = distance2;
            }

            //update motor speed
            motor1.motorSpeed = (float)(encoder1.ticks - encoder1.lastTicks);
            encoder1.lastTicks = encoder1.ticks;
            motor2.motorSpeed = (float)(encoder2.ticks - encoder2.lastTicks);
            encoder2.lastTicks = encoder2.ticks;

            //Compute Speed PID (input is wheel speed. output is angleSetpoint)
            speedPIDInput = (motor1.motorSpeed + motor2.motorSpeed)/2;
            speedPID.setSetpoint(userControl.direction);
            speedPIDOutput = speedPID.compute(speedPIDInput);

            //set angle setpoint and compensate to reach equilibrium point
            anglePID.setSetpoint(speedPIDOutput+configuration.calibratedZeroAngle);

            // update angle pid tuning. only update if different from current tuning
    		if((activePIDTuning == AGGRESSIVE) && (((anglePIDInput) > (-configuration.anglePIDLowerLimit+configuration.calibratedZeroAngle)) \
                                               && ((anglePIDInput) < (configuration.anglePIDLowerLimit+configuration.calibratedZeroAngle)))) {
    			//we're close to setpoint, use conservative tuning parameters
                activePIDTuning = CONSERVATIVE;
    			anglePID.setTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
    		}
    		else if ((activePIDTuning == CONSERVATIVE) && (((anglePIDInput) <= (-configuration.anglePIDLowerLimit+configuration.calibratedZeroAngle)) \
                                                       || ((anglePIDInput) >= (configuration.anglePIDLowerLimit+configuration.calibratedZeroAngle)))) {
    			//we're far from setpoint, use aggressive tuning parameters
                activePIDTuning = AGGRESSIVE;
    			anglePID.setTunings(configuration.anglePIDAggKp, configuration.anglePIDAggKi, configuration.anglePIDAggKd);
    		}

            if (((anglePIDInput) > (ANGLE_IRRECOVERABLE+configuration.calibratedZeroAngle)) || ((anglePIDInput) < (-ANGLE_IRRECOVERABLE+configuration.calibratedZeroAngle))){
    			// so sorry, we're licking the floor :(
                started = false;
    		}
/*
            // update angle pid tuning. only update if different from current tuning
            if((activePIDTuning == AGGRESSIVE) && (abs(anglePIDInput) < configuration.anglePIDLowerLimit)) {
                //we're close to setpoint, use conservative tuning parameters
                activePIDTuning = CONSERVATIVE;
                anglePID.setTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
            }
            else if ((activePIDTuning == CONSERVATIVE) && (abs(anglePIDInput) >= configuration.anglePIDLowerLimit)) {
                //we're far from setpoint, use aggressive tuning parameters
                activePIDTuning = AGGRESSIVE;
                anglePID.setTunings(configuration.anglePIDAggKp, configuration.anglePIDAggKi, configuration.anglePIDAggKd);
            }
            else if (abs(anglePIDInput) > ANGLE_IRRECOVERABLE){
                // so sorry, we're licking the floor :(
                started = false;
            }*/

            // Compute Angle PID (input is current angle, output is angleSetpoint)
    		anglePIDOutput = anglePID.compute(anglePIDInput);

            //Set PWM value
            if (started) {
                motor1.setSpeedPercentage(anglePIDOutput+userControl.steering);
                motor2.setSpeedPercentage(anglePIDOutput-userControl.steering);
            }
            else {
                motor1.motorOff();
                motor2.motorOff();
            }

            //write serial
            Serial.println(String(ori[0]) + "," + \
                           String(ori[1]) + "," + \
                           String(ori[2]) + "," + \
                           String(encoder1.ticks) + "," + \
                           String(encoder2.ticks) + "," + \
                           String(distance1) + "," + \
                           String(distance2) + "," + \
                           String(velocity1) + "," + \
                           String(velocity2) + "," + \
                           String(motor1.motorSpeed) + "," + \
                           String(motor2.motorSpeed) + "," + \
                           String(speedPIDOutput) + "," + \
                           String(anglePIDOutput) + "," + \
                           String(userControl.direction) + "," + \
                           String(userControl.steering));
        }
	}
}
