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

#define gravity 		256.0f
#define pi 				3.14159265359

#define Kp_ROLLPITCH 	0.02f
#define Ki_ROLLPITCH 	0.00002f
#define Kp_YAW 			1.2f
#define Ki_YAW 			0.00002f

#define CF 				0.95

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
	float gyroVector[3];
	float gyr_dt;
	float magnetoHeading(float magnetometer[3], float accelerometer[3]);

	// dcm sensor fusion algorithm
	float dcmMatrix[3][3];
	bool drift_correction;
	void dcmAlgorithm(float G_dt, bool driftCorrection, float (&orientationDeg)[3]);
	void complementaryFilder(float G_dt, float (&orientationDeg)[3]);
	void initRotationMatrix();
	void matrixUpdate();
	void normalize();
	void driftCorrection();
};


#endif
