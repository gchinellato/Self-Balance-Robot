
/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int16_t) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int16_t) 0x69) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
#define WIRE_SEND(b) Wire.write((byte) b)
#define WIRE_RECEIVE() Wire.read()
#else
#define WIRE_SEND(b) Wire.send(b)
#define WIRE_RECEIVE() Wire.receive()
#endif


void I2C_Init()
{
	Wire.begin();
}

void Accel_Init()
{
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x2D);  // Power register
	WIRE_SEND(0x08);  // Measurement mode
	Wire.endTransmission();
	delay(5);
	
	/*// Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x2C);  // Rate
	WIRE_SEND(0x09);  // Set to 50Hz, normal operation
	Wire.endTransmission();
	delay(5);*/
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
	int16_t i = 0;
	byte buff[6];
	
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x32);  // Send address to read from
	Wire.endTransmission();
	
	Wire.beginTransmission(ACCEL_ADDRESS);
	Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
	while(Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	
	if (i == 6)  // All bytes received?
	{
		// No multiply by -1 for coordinate system transformation here, because of double negation:
		// We want the gravity vector, which is negated acceleration vector.
		accel[0] = (int16_t) ((( buff[3]) << 8) | buff[2]);  // X axis (internal sensor y axis)
		accel[1] = (int16_t) ((( buff[1]) << 8) | buff[0]);  // Y axis (internal sensor x axis)
		accel[2] = (int16_t) ((( buff[5]) << 8) | buff[4]);  // Z axis (internal sensor z axis)
	}
	else
	{
		num_accel_errors++;
		if (output_errors) Serial.println("!ERR: reading accelerometer");
	}
}

void Magn_Init()
{
	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x02);
	WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
	Wire.endTransmission();
	delay(5);

	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x00);
	WIRE_SEND(0b00011000);  // Set 50Hz
	Wire.endTransmission();
	delay(5);
}

void Read_Magn()
{
	int16_t i = 0;
	byte buff[6];
	
	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x03);  // Send address to read from
	Wire.endTransmission();
	
	Wire.beginTransmission(MAGN_ADDRESS);
	Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
	while(Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	
	if (i == 6)  // All bytes received?
	{
		// 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
		#if HW__VERSION_CODE == 10125
		// MSB byte first, then LSB; X, Y, Z
		magnetom[0] = -1 * (int16_t) ((( buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
		magnetom[1] = -1 * (int16_t) ((( buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
		magnetom[2] = -1 * (int16_t) ((( buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
		// 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
		#elif HW__VERSION_CODE == 10736
		// MSB byte first, then LSB; Y and Z reversed: X, Z, Y
		magnetom[0] = -1 * (int16_t) ((( buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
		magnetom[1] = -1 * (int16_t) ((( buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
		magnetom[2] = -1 * (int16_t) ((( buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
		// 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
		#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
		// MSB byte first, then LSB; X, Y, Z
		magnetom[0] = (int16_t) ((( buff[0]) << 8) | buff[1]);         // X axis (internal sensor x axis)
		magnetom[1] = -1 * (int16_t) ((( buff[2]) << 8) | buff[3]));  // Y axis (internal sensor -y axis)
		magnetom[2] = -1 * (int16_t) ((( buff[4]) << 8) | buff[5]));  // Z axis (internal sensor -z axis)
		// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
		#elif HW__VERSION_CODE == 10724
		// MSB byte first, then LSB; Y and Z reversed: X, Z, Y
		magnetom[0] = (int16_t) (( buff[0]) << 8) | buff[1]);         // X axis (internal sensor x axis)
		magnetom[1] = -1 * (int16_t) ((( buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
		magnetom[2] = -1 * (int16_t) ((( buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
		#endif
	}
	else
	{
		num_magn_errors++;
		if (output_errors) Serial.println("!ERR: reading magnetometer");
	}
}

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

void writeRegister(int deviceAddress, byte address, byte val) 
{
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address)
{
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) 
    {
        // waiting
    }
    v = Wire.read();
    return v;
}

void Gyro_Init()
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:
  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
  // Rotina para leitura dos valores de X, Y e Z
  byte xMSB = readRegister(GYRO_ADDRESS, 0x29);
  byte xLSB = readRegister(GYRO_ADDRESS, 0x28);
  gyro[0] = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(GYRO_ADDRESS, 0x2B);
  byte yLSB = readRegister(GYRO_ADDRESS, 0x2A);
  gyro[1] = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(GYRO_ADDRESS, 0x2D);
  byte zLSB = readRegister(GYRO_ADDRESS, 0x2C);
  gyro[2] = ((zMSB << 8) | zLSB);

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
	int16_t i = 0;
	byte buff[6];
	
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x1D);  // Sends address to read from
	Wire.endTransmission();
	
	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
	while(Wire.available())  // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE();  // Read one byte
		i++;
	}
	Wire.endTransmission();
	
	if (i == 6)  // All bytes received?
	{
		gyro[0] = -1 * (int16_t) ((( buff[2]) << 8) | buff[3]);    // X axis (internal sensor -y axis)
		gyro[1] = -1 * (int16_t) ((( buff[0]) << 8) | buff[1]);    // Y axis (internal sensor -x axis)
		gyro[2] = -1 * (int16_t) ((( buff[4]) << 8) | buff[5]);    // Z axis (internal sensor -z axis)
	}
	else
	{
		num_gyro_errors++;
		if (output_errors) Serial.println("!ERR: reading gyroscope");
	}
}
