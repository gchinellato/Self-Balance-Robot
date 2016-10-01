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
#include "main.h"

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
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
    GY80 imu;

    while(1)
	{
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
