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
#include <Wire.h>
#include "IMU/GY80.h"
#include "Motion/Motor/motor.h"
#include "Motion/PID/PID.h"
#include "main.h"

Motor motor;

void setup()
{
    timestamp=millis();
    Serial.begin(SERIAL_BAUDRATE);

    Serial.println("setup");
    motor.Initialize();
/*
    GY80 imu;
	if (CALIBRATION_MAGNETO == 1)
	{
		Serial.println("MAGNETOMETER CALIBRATION STARTED");
		imu.magCalibration();
		Serial.println("CALIBRATION FINISHED");
	}
*/
}

void loop()
{
    Serial.println("loop");
    GY80 imu;
    while(1)
	{
        Serial.println("motor go CW");
        motor.motorGo(0, CW, 25);
        motor.motorGo(1, CW, 25);
        delay(10);

        Serial.println("motor go CCW");
        motor.motorGo(0, CCW, 25);
        motor.motorGo(1, CCW, 25);
        delay(10);

		if ((millis() - timestamp) >= DATA_INTERVAL)
		  {
			timestamp_old = timestamp;
			timestamp = millis();

			if (timestamp > timestamp_old)
			{
				G_Dt = (float)(timestamp - timestamp_old) / 1000.0f;
			}
			else
			{
				G_Dt = 0;
			}

			ori = imu.getOrientation(1, G_Dt); //pitch roll yaw
			Serial.println("Roll: " + String(ori[0]) + ", Pitch: " + String(ori[1]) + ", Yaw: " + String(ori[2]) + ", DT: " + String(G_Dt) );
        }
	}
}
