/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Gyro sensor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include "GY80_L3G4200D.h"

L3G4200D::L3G4200D()
{
	Wire.begin();

    //Enable gyro (Normal mode - X,Y,Z axes)
	writeTo(L3G4200D_CTRL_REG1, 0x0F);
    //Full scale selection (2000 dps)
    writeTo(L3G4200D_CTRL_REG4, 0x30);
}

void L3G4200D::writeTo(byte address, byte val)
{
	Wire.beginTransmission(L3G4200D_DEVICE);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

void L3G4200D::readFrom(byte address, int num, byte _buff[])
{
	Wire.beginTransmission(L3G4200D_DEVICE);
	Wire.write(address);
	Wire.endTransmission();

	Wire.beginTransmission(L3G4200D_DEVICE);
	Wire.requestFrom(L3G4200D_DEVICE, num);

	int i = 0;
	while(Wire.available())
	{
		_buff[i] = Wire.read();
		i++;
	}
	/*if(i != num){
		status = L3G4200D_ERROR;
		error_code = L3G4200D_READ_ERROR;
	}*/
	Wire.endTransmission();
}

void L3G4200D::read()
{
    byte X_L = 0;
    byte X_H = 0;
    byte Y_L = 0;
    byte Y_H = 0;
    byte Z_L = 0;
    byte Z_H = 0;

	readFrom(L3G4200D_OUT_X_L, 1, _buff);
    X_L = _buff[0];
    readFrom(L3G4200D_OUT_X_H, 1, _buff);
    X_H = _buff[0];
    readFrom(L3G4200D_OUT_Y_L, 1, _buff);
    Y_L = _buff[0];
    readFrom(L3G4200D_OUT_Y_H, 1, _buff);
    Y_H = _buff[0];
    readFrom(L3G4200D_OUT_Z_L, 1, _buff);
    Z_L = _buff[0];
    readFrom(L3G4200D_OUT_Z_H, 1, _buff);
    Z_H = _buff[0];

	gyroVector[0] = X_H << 8 | X_L;
	gyroVector[1] = Y_H << 8 | Y_L;
	gyroVector[2] = Z_H << 8 | Z_L;

    gyroVector[0] = getSignedNumber(gyroVector[0]);
    gyroVector[1] = getSignedNumber(gyroVector[1]);
    gyroVector[2] = getSignedNumber(gyroVector[2]);

	//gyroVector[0] *= -1;
	//gyroVector[1] *= -1;
	//gyroVector[2] *= -1;
}

int L3G4200D::getSignedNumber(int number)
{
    if(number & (1 << 15))
    {
        number |= !0xFFFF;
    }
    else
    {
        number &= 0xFFFF;
    }
    return number;
}

void L3G4200D::scale()
{
	rateVector[0] = 0;
	rateVector[1] = 0;
	rateVector[2] = 0;

	rateVector[0] = gyroVector[0] * GYRO_SENSITIVITY;
	rateVector[1] = gyroVector[1] * GYRO_SENSITIVITY;
	rateVector[2] = gyroVector[2] * GYRO_SENSITIVITY;
}

void L3G4200D::applyCalibration()
{

}

/*
 * getOrientationVector method:
 * store calibrated and scaled accelerometer data(x,y,z) in '&data' parameter
 */
void L3G4200D::getGyroVector(float (&data)[3])
{
	read();
	applyCalibration();
	scale();
	//Serial.println("X: " + String(gyroVector[0]) + ",Y: " + String(gyroVector[1]) + ",Z: " + String(gyroVector[2]));

	memcpy(data, gyroVector, sizeof(data));
}
