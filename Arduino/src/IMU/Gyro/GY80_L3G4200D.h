/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Gyro sensor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#ifndef L3G4200D_H
#define L3G4200D_H

#include <Arduino.h>

#define L3G4200D_DEVICE            0x69 // I2C address, 0b11010001
#define L3G4200D_CTRL_REG1         0x20 // Enable Power and X,Y,Z axes
#define L3G4200D_CTRL_REG2         0x21 // Full scale selection
#define L3G4200D_CTRL_REG3         0x22
#define L3G4200D_CTRL_REG4         0x23
#define L3G4200D_CTRL_REG5         0x24
#define L3G4200D_OUT_X_L           0x28 // X-axis data 0
#define L3G4200D_OUT_X_H           0x29 // X-axis data 1
#define L3G4200D_OUT_Y_L           0x2A // Y-axis data 0
#define L3G4200D_OUT_Y_H           0x2B // Y-axis data 1
#define L3G4200D_OUT_Z_L           0x2C // Z-axis data 0
#define L3G4200D_OUT_Z_H           0x2D // Z-axis data 1
#define GYRO_SENSITIVITY           0.07 // 2000dps datasheet
#define L3G4200D_TO_READ 	       6

class L3G4200D
{
public:
	L3G4200D();
	void getGyroVector(float (&data)[3]);
	float rateVector[3]; // x, y, z
private:
	float gyroVector[3]; // x, y, z
	float offset[3];
	void read();
	void scale();
    int getSignedNumber(int number);
	void applyCalibration();
	void writeTo(byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);
	byte _buff[6];
};

#endif
