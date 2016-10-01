// Programa : Teste HMC5883L - Bussola
// Adaptacoes : Arduino e Cia

#include <Wire.h>

#define Register_ID 0
#define Register_2D 0x2D
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37
#define ADXL345_DEVICE 0x53

// Endereco I2C do sensor : 83 em decimal ou 0x53
int ADXAddress = 0x53;  // the default 7-bit slave address
int reading = 0;
int val=0;
int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
double Xg,Yg,Zg;


void I2C_Init()
{
	Wire.begin();
}

void initAccel()
{
    // enable to measute g data
    Wire.beginTransmission(ADXL345_DEVICE);
    Wire.write(Register_2D); // Power register
    Wire.write(8);                //measuring enable
    Wire.endTransmission();     // stop transmitting
	delay(5);
}

// Reads x, y and z accelerometer registers
void readAccel()
{
	int16_t i = 0;
	byte buff[6];
	
	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.write(Register_X0);  // Send address to read from
	Wire.endTransmission();
	
	Wire.beginTransmission(ADXL345_DEVICE);
	Wire.requestFrom(ADXL345_DEVICE, 6);  // Request 6 bytes
	while(Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = Wire.read();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	
	if (i == 6)  // All bytes received?
	{
		// No multiply by -1 for coordinate system transformation here, because of double negation:
		// We want the gravity vector, which is negated acceleration vector.
		accel[0] = (int16_t) ((( buff[0]) << 8) | buff[1]);  // X axis (internal sensor y axis)
		accel[1] = (int16_t) ((( buff[2]) << 8) | buff[3]);  // Y axis (internal sensor x axis)
		accel[2] = (int16_t) ((( buff[4]) << 8) | buff[4]);  // Z axis (internal sensor z axis)
	}
	else
	{
		num_accel_errors++;
		if (output_errors) Serial.println("!ERR: reading accelerometer");
	}
}

void setup()
{
    initI2C()
    initAccel()
}

void loop()
{
    readSensors()
}
