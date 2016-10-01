/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: GY80 IMU sensor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "GY80.h"

GY80::GY80()
{
    delay(50); // time for sensors to start
}

void GY80::magCalibration()
{
	float magRange[6]; // MAG_MAX_X; MAG_MIN_Y; MAG_MAX_Y; MAG_MIN_Y; MAG_MAX_Z; MAG_MIN_Z
	int calibration_counter=0;
	int EEPROM_ADDRESS=0; // eeprom offset for mag range values

	while(calibration_counter<3000)
	{
		Serial.println(String(calibration_counter));
		magnetometer.getMagVector(magVector); // -----> need function to read rawData

		Serial.println("X max: " + String(magRange[0]) + ", min: " + String(magRange[1]));
        Serial.println("Y max: " + String(magRange[2]) + ", min: " + String(magRange[3]));
        Serial.println("Z max: " + String(magRange[4]) + ", min: " + String(magRange[5]));
		if (magVector[0] > magRange[0]) { magRange[0] = magVector[0]; } //max x
		if (magVector[0] < magRange[1]) { magRange[1] = magVector[0]; } //min x
		if (magVector[1] > magRange[2]) { magRange[2] = magVector[1]; } //max y
		if (magVector[1] < magRange[3]) { magRange[3] = magVector[1]; } //min y
		if (magVector[2] > magRange[4]) { magRange[4] = magVector[2]; } //max z
		if (magVector[2] < magRange[5]) { magRange[5] = magVector[2]; } //min z

		delay(10);
		calibration_counter++;
	}

	for (int i=0; i<6; i++)
	{
        Serial.println("Range: " + String(i) + " " + String(magRange[i]));
		EEPROM.put(EEPROM_ADDRESS + (i*(sizeof(float))), magRange[i]);
	}
}

float* GY80::getOrientation(int algorithm, float G_dt)
{
    //dcmAlgorithm(G_dt, false, orientation); 	// pitch, roll, yaw
    complementaryFilder(G_dt, orientation); // roll, pitch, yaw
	return orientation;
}

void GY80::complementaryFilder(float G_dt, float (&orientationDeg)[3])
{
    accelerometer.getAccVector(accVector);

    gyro.getGyroVector(gyroVector);

    magnetometer.getMagVector(magVector);
    magnetometer.tiltCompensation(accelerometer.roll, accelerometer.pitch, compMagVector);
    magnetometer.setDeclination(-21.0, 7.0);
    magnetometer.getHeading(compMagVector[0], compMagVector[1], true);

    // CF = tau / (tau+LP)
    // tau = CF*LP/(1-CF)
    //i.e: 0.98*0.01sec/(1-0.98) = 0.49sec
    //(if the loop period is shorter than this value, gyro take precedence, otherwise, acceleromter is given more weighting than gyro)
    // orientation in degrees (pitch, roll, yaw from rotation matrix)
    orientationDeg[0] = CF*(orientationDeg[0] + gyro.rateVector[0]*G_dt) + (1-CF)*accelerometer.roll*RAD_TO_DEG;
    orientationDeg[1] = CF*(orientationDeg[1] + gyro.rateVector[1]*G_dt) + (1-CF)*accelerometer.pitch*RAD_TO_DEG;
    orientationDeg[2] = CF*(orientationDeg[2] + gyro.rateVector[2]*G_dt) + (1-CF)*magnetometer.heading*RAD_TO_DEG;
}
