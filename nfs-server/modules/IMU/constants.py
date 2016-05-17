"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: GY80 Header - Orientation sensor via I2C bus 
*               HMC5883L (3-Axis Digital Compass)
*               ADXL345 (3-Axis Digital Accelerometer)
*               L3G4200D (3-Axis Angular Rate Sensor / Gyro)
*               BMP085 (Barometric Pressure / Temperature Sensor)
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""
import math

#
#L3G42000D Gyro Registers	
#
L3G42000D_ADDRESS          = 0x69 #I2C address, 0b11010001
L3G42000D_CTRL_REG1        = 0x20 #Enable Power and X,Y,Z axes
L3G42000D_CTRL_REG2        = 0x21 #Full scale selection
L3G42000D_CTRL_REG3        = 0x22
L3G42000D_CTRL_REG4        = 0x23
L3G42000D_CTRL_REG5        = 0x24
L3G42000D_OUT_X_L          = 0x28 #X-axis data 0
L3G42000D_OUT_X_H          = 0x29 #X-axis data 1
L3G42000D_OUT_Y_L          = 0x2A #Y-axis data 0
L3G42000D_OUT_Y_H          = 0x2B #Y-axis data 1
L3G42000D_OUT_Z_L          = 0x2C #Z-axis data 0
L3G42000D_OUT_Z_H          = 0x2D #Z-axis data 1
GYRO_SENSITIVITY           = 0.07 # 2000dps datasheet

#
#ADXL345 Accel Registers	
#
ADXL345_ADDRESS          = 0x53 #I2C address
ADXL345_REG_DEVID        = 0x00 # Device ID
ADXL345_REG_DATAX0       = 0x32 # X-axis data 0 (6 bytes for X/Y/Z)
ADXL345_REG_POWER_CTL    = 0x2D # Power-saving features control
ADXL345_DATARATE_0_10_HZ = 0x00
ADXL345_DATARATE_0_20_HZ = 0x01
ADXL345_DATARATE_0_39_HZ = 0x02
ADXL345_DATARATE_0_78_HZ = 0x03
ADXL345_DATARATE_1_56_HZ = 0x04
ADXL345_DATARATE_3_13_HZ = 0x05
ADXL345_DATARATE_6_25HZ  = 0x06
ADXL345_DATARATE_12_5_HZ = 0x07
ADXL345_DATARATE_25_HZ   = 0x08
ADXL345_DATARATE_50_HZ   = 0x09
ADXL345_DATARATE_100_HZ  = 0x0A # (default)
ADXL345_DATARATE_200_HZ  = 0x0B
ADXL345_DATARATE_400_HZ  = 0x0C
ADXL345_DATARATE_800_HZ  = 0x0D
ADXL345_DATARATE_1600_HZ = 0x0E
ADXL345_DATARATE_3200_HZ = 0x0F
ADXL345_RANGE_2_G        = 0x00 # +/-  2g (default)
ADXL345_RANGE_4_G        = 0x01 # +/-  4g
ADXL345_RANGE_8_G        = 0x02 # +/-  8g
ADXL345_RANGE_16_G       = 0x03 # +/- 16g
ACC_SCALE_MULTIPLIER     = 0.004 #scale 255=1g=9.81m/s2 1/255=0.0004

#
#BMP085 Barometer Registers
#
BMP085_ADDRESS           = 0x77 #I2C address
BMP085_CAL_AC1           = 0xAA  # R   Calibration data (16 bits)
BMP085_CAL_AC2           = 0xAC  # R   Calibration data (16 bits)
BMP085_CAL_AC3           = 0xAE  # R   Calibration data (16 bits)
BMP085_CAL_AC4           = 0xB0  # R   Calibration data (16 bits)
BMP085_CAL_AC5           = 0xB2  # R   Calibration data (16 bits)
BMP085_CAL_AC6           = 0xB4  # R   Calibration data (16 bits)
BMP085_CAL_B1            = 0xB6  # R   Calibration data (16 bits)
BMP085_CAL_B2            = 0xB8  # R   Calibration data (16 bits)
BMP085_CAL_MB            = 0xBA  # R   Calibration data (16 bits)
BMP085_CAL_MC            = 0xBC  # R   Calibration data (16 bits)
BMP085_CAL_MD            = 0xBE  # R   Calibration data (16 bits)
BMP085_CONTROL           = 0xF4
BMP085_TEMPDATA          = 0xF6
BMP085_PRESSUREDATA      = 0xF6
BMP085_READTEMPCMD       = 0x2E
BMP085_READPRESSURECMD   = 0x34
# Operating Modes
BMP085_ULTRALOWPOWER     = 0
BMP085_STANDARD          = 1
BMP085_HIGHRES           = 2
BMP085_ULTRAHIGHRES      = 3

#
#HMC5883L Compass Registers
#
HMC5883L_ADDRESS                 = 0x1E #I2C address
HMC5883L_ConfigurationRegisterA  = 0x00
HMC5883L_ConfigurationRegisterB  = 0x01
HMC5883L_ModeRegister            = 0x02
HMC5883L_AxisXDataRegisterMSB    = 0x03
HMC5883L_AxisXDataRegisterLSB    = 0x04
HMC5883L_AxisZDataRegisterMSB    = 0x05
HMC5883L_AxisZDataRegisterLSB    = 0x06
HMC5883L_AxisYDataRegisterMSB    = 0x07
HMC5883L_AxisYDataRegisterLSB    = 0x08
HMC5883L_StatusRegister          = 0x09
HMC5883L_IdentificationRegisterA = 0x10
HMC5883L_IdentificationRegisterB = 0x11
HMC5883L_IdentificationRegisterC = 0x12
#Operations Modes
HMC5883L_MeasurementContinuous = 0x00
HMC5883L_MeasurementSingleShot = 0x01
HMC5883L_MeasurementIdle = 0x03

#
#Others
#
RAD_TO_DEG = 180.0/math.pi
DEG_TO_RAD = math.pi/180.0
EARTH_GRAVITY_MS2   = 9.80665 # earth acceleration
CF =  0.95      # Complementary filter constant 
X = 0
Y = 1
Z = 2
#magOffsetPath = "magnetometer_calibration_offsets.txt"
magOffsetPath = "modules/IMU/Mag/magnetometer_calibration_offsets.txt"
