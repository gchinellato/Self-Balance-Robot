#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: GY80 API - IMU sensor API
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                             
*************************************************
"""

from IMU.Gyro.GY80_L3G4200D import GY80_Gyro_L3G42000D
from IMU.Accel.GY80_ADXL345 import GY80_Accelerometer_ADXL345
from IMU.Mag.GY80_HMC5883L import GY80_Magnetometer_HMC5883L
from constants import *
from Utils.traces.trace import *

class GY80_IMU():
    def __init__(self, debug=False, magCal=False):
        self.debug = debug

        #Enable Accelerometer, Gyro and Mag
        self.accel = GY80_Accelerometer_ADXL345()
        self.gyro = GY80_Gyro_L3G42000D()
        self.mag = GY80_Magnetometer_HMC5883L(cal=magCal)

        #Complementary filter angles
        self.CFangleRoll = 0.0
        self.CFanglePitch = 0.0
        self.CFangleYaw = 0.0

        logging.info("GY80 IMU module initialized") 

    def _readAcc(self):
        #
        # Accelerometer
        #

        # Read accel values
        rawAcc = self.accel.readRawAcc()
        scaledAcc = self.accel.scaleAcc(rawAcc)
        self.accel.calculateAngles(scaledAcc)

    def _readGyro(self):
        #
        # Gyroscope
        #

        # Read gyro values
        rawGyro = self.gyro.readRawGyro()
        self.gyro.scaleGyro(rawGyro)

    def _readMag(self):
        #
        # Magnetonometer
        #

        # Magnetic declination 
        # Santo Andre @ Brazil:
        # Declination: -21 7 (Negative WEST)
        self.mag.setDeclination(-21,7) 

        # Read mag values
        rawMag = self.mag.readRawMag()
        calMag = self.mag.hardCalibration(rawMag, self.mag.magOffset)
        scaledMag = self.mag.scaleMag(calMag)         
        compMag = self.mag.tiltCompensation(self.accel.Roll, self.accel.Pitch, scaledMag)   
        self.mag.getHeading(compMag[X], compMag[Y]) 

    def readIMU(self):
        self._readAcc()
        self._readGyro()
        self._readMag()
         
    def complementaryFilter(self, CF, LP):
        #
        # Complementary Filter
        #

        # Complementary Filter used to combine the accelerometer/mag and gyro values to result the Euler Angles
        # Gyro has a BIAS (drift), then it is only reliable on short term. 
        # Accelerometer has noise due any disturb or vibration, then it is only reliable on long term.

        #Read IMU sensors
        self.readIMU()

        # CF = tau / (tau+LP)
        # tau = CF*LP/(1-CF)
        #i.e: 0.98*0.01sec/(1-0.98) = 0.49sec 
        # (if the loop period is shorter than this value, gyro take precedence, otherwise, acceleromter is given more weighting than gyro)
        self.CFangleRoll = CF*(self.CFangleRoll + self.gyro.rateGyro[X]*LP) + (1 - CF) * self.accel.Roll*RAD_TO_DEG
        self.CFanglePitch = CF*(self.CFanglePitch + self.gyro.rateGyro[Y]*LP) + (1 - CF) * self.accel.Pitch*RAD_TO_DEG
        self.CFangleYaw = CF*(self.CFangleYaw + self.gyro.rateGyro[Z]*LP) + (1 - CF) * self.mag.Heading  

        self.CFangleRoll = round(self.CFangleRoll, 2)
        self.CFanglePitch = round(self.CFanglePitch, 2)
        self.CFangleYaw = round(self.CFangleYaw, 2)

        if (self.debug):
            logging.debug(("Filter: Roll(x): %0.2f, Pitch(y): %0.2f, Yaw(z): %0.2f" % (self.CFangleRoll, self.CFanglePitch, self.CFangleYaw)))   
 
    def quaternion(self):
        #
        # Quaternion 
        #

        #Read IMU sensors
        self.readIMU()

