/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Magnetometer sensor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "GY80_HMC5883L.h"

/*
 * object constructor:
 * initialize I2C communication
 * write 0x00 to register 0x01 (place the part in continuous measurement mode)
 * write 0x70 to register 0x00 (Configuration Register A - default configuration)
 * write appropriate value depending on HMC5883L_DefaultFieldRange to register 0x01(configuration register B)
 */
HMC5883L::HMC5883L()
{
	Wire.begin();

	writeTo(HMC5883L_ModeRegister, HMC5883L_MeasurementContinuous);
	writeTo(HMC5883L_ConfigurationRegisterA, 0x18);

	if (HMC5883L_DefaultFieldRange==0.88) // Nominal gain configuration (HMC5883L_ConfigurationRegisterB)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 1370.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x00);
	}
	else if (HMC5883L_DefaultFieldRange==1.3)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 1090.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x20);
	}
	else if (HMC5883L_DefaultFieldRange==1.9)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 820.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x40);
	}
	else if (HMC5883L_DefaultFieldRange==2.5)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 660.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x60);
	}
	else if (HMC5883L_DefaultFieldRange==4.0)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 440.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x80);
	}
	else if (HMC5883L_DefaultFieldRange==4.7)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 390.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0xA0);
	}
	else if (HMC5883L_DefaultFieldRange==5.6)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 330.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0xC0);
	}
	else if (HMC5883L_DefaultFieldRange==8.1)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 230.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0xE0);
	}
	else // out of range - return to defaults // default configuration: field range 1.3Ga
	{
		HMC5883L_SCALE_FACTOR = (1000 / 1090);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x20);
	}
}

/*
 * writeTo method:
 * write 'val' in 'address' register (using I2C bus <Wire> lib)
 */
void HMC5883L::writeTo(byte address, byte val)
{
	Wire.beginTransmission(HMC5883L_DEVICE);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

/*
 * readFrom method:
 * read 'num' bytes from starting 'address' register onwards and stores in '_buff[]'
 * (using I2C bus <Wire> lib)
 */
void HMC5883L::readFrom(byte address, int num, byte _buff[])
{
	Wire.beginTransmission(HMC5883L_DEVICE);
	Wire.write(address);
	Wire.endTransmission();

	Wire.beginTransmission(HMC5883L_DEVICE);
	Wire.requestFrom(HMC5883L_DEVICE, num);

	int i = 0;
	while(Wire.available())
	{
		_buff[i] = Wire.read();
		i++;
	}
	/*if(i != num){
		status = HMC5883L_ERROR;
		error_code = HMC5883L_READ_ERROR;
	}*/
	Wire.endTransmission();
}

/*
 * setGain method:
 * set HMC5883L_SCALE_FACTOR based on 'fieldRange' value
 * write to HMC5883L_ConfigurationRegisterB register the appropriate value for the specified 'fieldRange'
 */
void HMC5883L::setGain(float fieldRange)
{
	if (fieldRange==0.88) // Nominal gain configuration (HMC5883L_ConfigurationRegisterB)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 1370.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x00);
	}
	else if (fieldRange==1.3)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 1090.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x20);
	}
	else if (fieldRange==1.9)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 820.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x40);
	}
	else if (fieldRange==2.5)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 660.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x60);
	}
	else if (fieldRange==4.0)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 440.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x80);
	}
	else if (fieldRange==4.7)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 390.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0xA0);
	}
	else if (fieldRange==5.6)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 330.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0xC0);
	}
	else if (fieldRange==8.1)
	{
		HMC5883L_SCALE_FACTOR = (1000.0f / 230.0f);
		writeTo(HMC5883L_ConfigurationRegisterB, 0xE0);
	}
	else // out of range - return to defaults // default configuration: field range 1.3Ga
	{
		HMC5883L_SCALE_FACTOR = (1000 / 1090);
		writeTo(HMC5883L_ConfigurationRegisterB, 0x20);
	}
}

/*
 * read method:
 * store sensor data(x,y,z) in 'orientationVector'
 *
 * values in two's complement form    // X, Z, Y
	 * X value: _buff[1] is MSB & _buff[0] is LSB
	 * Y value: _buff[4] is MSB & _buff[5] is LSB
	 * Z value: _buff[2] is MSB & _buff[3] is LSB
 */
void HMC5883L::read()
{
	readFrom(HMC5883L_AxisXDataRegisterMSB, HMC5883L_TO_READ, _buff); //Read HMC5883L_TO_READ bytes from HMC5883L_AxisXDataRegisterMSB onwards and store it in _buff

	magVector[0] = (((int)_buff[0]) << 8) | _buff[1]; // X
	magVector[2] = (((int)_buff[2]) << 8) | _buff[3]; // Z
	magVector[1] = (((int)_buff[4]) << 8) | _buff[5]; // Y

	//magVector[0] *= -1;
	//magVector[1] *= -1;
	//magVector[2] *= -1;
}

/*
 * scale method:
 * apply HMC5883L_SCALE_FACTOR to 'orientationVector' data
 */
void HMC5883L::scaleGain()
{
	magVector[0] *= HMC5883L_SCALE_FACTOR;
	magVector[1] *= HMC5883L_SCALE_FACTOR;
	magVector[2] *= HMC5883L_SCALE_FACTOR;
}

/*
 * applyCalibration method:
 * read max/min range values from EEPROM memory sector and apply offset to 'orientationVector' data
 *
 * range values stored in EEPROM by calibration algorithm
 */
void HMC5883L::applyCalibration()
{
	int EEPROM_ADDRESS=0;  // eeprom offset for magnetometer range values
	float valueRange[6]; //MAG_X_MAX, MAG_X_MIN, MAG_Y_MAX, MAG_Y_MIN, MAG_Z_MAX, MAG_Z_MIN

	for(int i=0;i<6;i++)
	{
		EEPROM.get(EEPROM_ADDRESS + (i*(sizeof(float))),valueRange[i]);
	}

	float offsetX = (valueRange[0] + valueRange[1]) / 2.0f;
	float offsetY = (valueRange[2] + valueRange[3]) / 2.0f;
	float offsetZ = (valueRange[4] + valueRange[5]) / 2.0f;

	//MAGN_X_SCALE = (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
	float mag_x_scale = (100.0f / (valueRange[0] - offsetX));
	float mag_y_scale = (100.0f / (valueRange[2] - offsetY));
	float mag_z_scale = (100.0f / (valueRange[4] - offsetZ));

	// magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
	magVector[0] = (magVector[0] - offsetX) * mag_x_scale;
	magVector[1] = (magVector[1] - offsetY) * mag_y_scale;
	magVector[2] = (magVector[2] - offsetZ) * mag_z_scale;
}

void HMC5883L::tiltCompensation(float accRoll, float accPitch, float (&data)[3])
{
	float cosPitch = cos(accPitch);
	float sinPitch = sin(accPitch);
	float cosRoll = cos(accRoll);
	float sinRoll = sin(accRoll);

	//Tilt Compensation
	float compMag[3] = {0,0,0};
	compMag[0] = (magVector[0]*cosPitch) + (magVector[2]*sinPitch);
	compMag[1] = (magVector[0]*sinRoll*sinPitch) + (magVector[1]*cosRoll) - (magVector[2]*sinRoll*cosPitch);
	compMag[2] = (-magVector[0]*cosRoll*sinPitch) + (magVector[1]*sinRoll) - (magVector[2]*cosRoll*cosPitch);

	memcpy(data, compMag, sizeof(data));
}

void HMC5883L::setDeclination(float degree, float minutes)
{
	declination = (degree+minutes/60)*(M_PI/180);
}

void HMC5883L::getHeading(float compX, float compY, boolean declinationFlag)
{
		float headingRad;

		//Only needed if the heading value does not increase when the magnetometer is rotated clockwise
		//compY = -compY;

		headingRad = atan2(compY, compX);

		if (declinationFlag == true)
		{
			headingRad += declination;
		}

		//Correct for reversed heading
		if (headingRad < 0)
			headingRad += 2*M_PI;

		//Check for wrap and compensate
		if (headingRad > 2*M_PI)
			headingRad -= 2*M_PI;

		heading = headingRad;
}

/*
 * getOrientationVector method:
 * store calibrated and scaled accelerometer data(x,y,z) in '&data' parameter
 */
void HMC5883L::getMagVector(float (&data)[3])
{
	read();
	applyCalibration();
	scaleGain();

	memcpy(data, magVector, sizeof(data));
}
