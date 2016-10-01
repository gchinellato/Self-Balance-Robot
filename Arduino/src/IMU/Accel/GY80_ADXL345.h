/**
*************************************************
* @Project: Self Balance
* @Platform: Arduino Nano ATmega328
* @Description: Accelerometer sensor
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
 */

#ifndef ADXL345_H
#define ADXL345_H

#include <Arduino.h>

#define ADXL345_DEVICE 			0x53 // I2C addr
#define ADXL345_DEVID 			0x00 // Device ID
#define ADXL345_POWER_CTL       0x2D // Power-saving features control
#define ADXL345_DATAX0 			0x32 // X-axis data 0 (6 bytes for X/Y/Z)
#define ADXL345_DATAX1 			0x33
#define ADXL345_DATAY0 			0x34
#define ADXL345_DATAY1 			0x35
#define ADXL345_DATAZ0 			0x36
#define ADXL345_DATAZ1 			0x37
// num of bytes we are going to read each time (two bytes for each axis)
#define ADXL345_TO_READ 		6

#define ADXL345_DATARATE_0_10_HZ  0x00
#define ADXL345_DATARATE_0_20_HZ  0x01
#define ADXL345_DATARATE_0_39_HZ  0x02
#define ADXL345_DATARATE_0_78_HZ  0x03
#define ADXL345_DATARATE_1_56_HZ  0x04
#define ADXL345_DATARATE_3_13_HZ  0x05
#define ADXL345_DATARATE_6_25HZ   0x06
#define ADXL345_DATARATE_12_5_HZ  0x07
#define ADXL345_DATARATE_25_HZ    0x08
#define ADXL345_DATARATE_50_HZ    0x09
#define ADXL345_DATARATE_100_HZ   0x0A //(default)
#define ADXL345_DATARATE_200_HZ   0x0B
#define ADXL345_DATARATE_400_HZ   0x0C
#define ADXL345_DATARATE_800_HZ   0x0D
#define ADXL345_DATARATE_1600_HZ  0x0E
#define ADXL345_DATARATE_3200_HZ  0x0F
#define ADXL345_RANGE_2_G         0x00  // +/-  2g (default)
#define ADXL345_RANGE_4_G         0x01  // +/-  4g
#define ADXL345_RANGE_8_G         0x02  // +/-  8g
#define ADXL345_RANGE_16_G        0x03  // +/- 16g
#define ACC_SCALE_MULTIPLIER      0.004 //scale 255=1g=9.81m/s2 1/255=0.0004
#define EARTH_GRAVITY_MS2         9.80665 // earth acceleration

class ADXL345
{
public:
	ADXL345();
	void getAccVector(float (&data)[3]);
	float pitch;
	float roll;
private:
	float accVector[3]; // x, y, z
	float offset[3];
	void read();
    int getSignedNumber(int number);
	void scale(int mode);
	void applyCalibration();
	void getAccAngles();
	void writeTo(byte address, byte val);
	void readFrom(byte address, int num, byte buff[]);
	byte _buff[6];
};

#endif
