/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: GY80 IMU sensor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#ifndef GY80_H
#define GY80_H

#include "Accel/GY80_ADXL345.h"
#include "Gyro/GY80_L3G4200D.h"
#include "Mag/GY80_HMC5883L.h"

#define CF 				0.96

class GY80
{
public:
	GY80();
	float* getOrientation(int algorithm, float G_dt);

	void magCalibration();
	void accCalibration();
	void gyrCalibration();
private:
	ADXL345 accelerometer;
	L3G4200D gyro;
	HMC5883L magnetometer;
	float orientation[3]; //pitch, roll, yaw
	float accVector[3];
	float magVector[3];
	float compMagVector[3];
	float gyroVector[3];
	float magnetoHeading(float magnetometer[3], float accelerometer[3]);
	void complementaryFilder(float G_dt, float (&orientationDeg)[3]);
};


#endif
