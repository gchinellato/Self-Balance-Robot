#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: GY80 API - Orientation sensor via I2C bus
*               HMC5883L (Magnetometer/Compass sensor)
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                               
*************************************************
"""

from Utils.i2c.Adafruit_I2C import Adafruit_I2C
from IMU.Accel.GY80_ADXL345 import GY80_Accelerometer_ADXL345
from IMU.constants import *
import math
import time
import os
import sys
from Utils.traces.trace import *

class GY80_Magnetometer_HMC5883L():
    def __init__(self, gauss=1.3, busnum=-1, cal=False, debug=False):
        self.i2c = Adafruit_I2C(HMC5883L_ADDRESS, busnum, debug)
        self.debug = debug
        self.gauss = gauss
        self.cal = cal

        self.magOffset = None
        self.scale = None
        self.declination = None
        self.Heading = None

        #Set to Data output rate @ 75Hz
        self.i2c.write8(HMC5883L_ConfigurationRegisterA, 0x18)
        #Set 1.3 gain LSb / Gauss 1090 (default)
        self.setScale(gauss)        
        #Continuous sampling
        self.i2c.write8(HMC5883L_ModeRegister, HMC5883L_MeasurementContinuous)

        #Hard calibration offset    
        if cal == True:
            self.magOffset = self.getOffset()
            raw_input("Press any key to continue...")
        else:
            #read offset from a file
            self.magOffset = self.restoreOffsetCalibration()

        logging.info("Magnetometer module initialized")

    def setScale(self, gauss):
        if gauss == 0.88:
			self.scale = 0.73
        elif gauss == 1.3:
			scale_reg = 0x01
			self.scale = 0.92
        elif gauss == 1.9:
			scale_reg = 0x02
			self.scale = 1.22
        elif gauss == 2.5:
			scale_reg = 0x03
			self.scale = 1.52
        elif gauss == 4.0:
			scale_reg = 0x04
			self.scale = 2.27
        elif gauss == 4.7:
			scale_reg = 0x05
			self.scale = 2.56
        elif gauss == 5.6:
			scale_reg = 0x06
			self.scale = 3.03
        elif gauss == 8.1:
			scale_reg = 0x07
			self.scale = 4.35
		
        scale_reg = scale_reg << 5
        self.i2c.write8(HMC5883L_ConfigurationRegisterB , scale_reg)
	
    #Read raw data
    def readRawMag(self, scaled=False):
        bytes = self.i2c.readList(HMC5883L_AxisXDataRegisterMSB, 6)

        mag = []
        #read x,y,z raw data
        for i in range(0, 6, 2):
            #merge MSB|LSB
            g = (bytes[i] << 8) | bytes[i+1]
            #negative values
            if g > 32767:
                g -= 65536
            mag.append(g)

        #According datasheet, read sequence is X, Z and Y
        rawMag = [0,0,0]
        rawMag[X] = mag[0]
        rawMag[Y] = mag[2]
        rawMag[Z] = mag[1]

        if (self.debug):
            logging.debug(("Mag Raw: x:%d, y:%d, z:%d" % (rawMag[X], rawMag[Y], rawMag[Z])))

        return rawMag

    #Normalize data: v'= v/\v\ - unit vector
    def normalizeRawData(self, rawMag):
        normMag = [0,0,0]
        normMag[X] = rawMag[X]/math.sqrt(rawMag[X]*rawMag[X] + rawMag[Y]*rawMag[Y] + rawMag[Z]*rawMag[Z])
        normMag[Y] = rawMag[Y]/math.sqrt(rawMag[X]*rawMag[X] + rawMag[Y]*rawMag[Y] + rawMag[Z]*rawMag[Z])
        normMag[Z] = rawMag[Z]/math.sqrt(rawMag[X]*rawMag[X] + rawMag[Y]*rawMag[Y] + rawMag[Z]*rawMag[Z])

        if (self.debug):
            logging.debug(("Mag Normalized: x:%.3f, y:%.3f, z:%.3f" % (normMag[X], normMag[Y], normMag[Z])))

        return normMag

    #Convert raw mag to Gauss
    def scaleMag(self, rawMag):        
        #In the event the ADC reading overflows or underflows for the given channel, or if there is a math overflow during the bias
        #measurement, this data register will contain the value -4096.
        scaledMag = [0,0,0]
        if (rawMag[X] == -4096):
		    scaledMag[X] = None
        else:
		    scaledMag[X] = round(rawMag[X] * self.scale, 4)
		
        if (rawMag[Y] == -4096):
		    scaledMag[Y] = None
        else:
		    scaledMag[Y] = round(rawMag[Y] * self.scale, 4)
		
        if (rawMag[Z] == -4096):
		    scaledMag[Z] = None
        else:
		    scaledMag[Z] = round(rawMag[Z] * self.scale, 4)

        if (self.debug):
            logging.debug(("Mag Scaled: x:%.3f, y:%.3f, z:%.3f (mG)" % (scaledMag[X], scaledMag[Y], scaledMag[Z])))

        return scaledMag
	
    # Set actual declination according localization
    def setDeclination(self, degree, minutes):
		self.declination = (degree+minutes/60) * (math.pi/180)

    # Offset calibration - Get max and min values from calibration 
    def getOffset(self):
        magXmax = 0
        magYmax = 0
        magZmax = 0
        magXmin = 0
        magYmin = 0
        magZmin = 0

        logging.info("Mag calibration")
        raw_input("Rotate the device through the 3-axes. Press any key to start...")
        
        for i in range(300):
            rawMag = self.readRawMag()
            logging.info(("Calibrating Mag...%d: " % (i)))
            logging.info(("MAG X MAX: %0.3f MIN %0.3f " % (magXmax, magXmin)))
            logging.info(("MAG Y MAX: %0.3f MIN %0.3f " % (magYmax, magYmin)))
            logging.info(("MAG Z MAX: %0.3f MIN %0.3f " % (magZmax, magZmin)))

            if (rawMag[X] > magXmax):
                magXmax = rawMag[X]
            if (rawMag[Y] > magYmax):
                magYmax = rawMag[Y]
            if (rawMag[Z] > magZmax):
                magZmax = rawMag[Z]
            if (rawMag[X] < magXmin):
                magXmin = rawMag[X]
            if (rawMag[Y] < magYmin):
                magYmin = rawMag[Y]
            if (rawMag[Z] < magZmin):
                magZmin = rawMag[Z]

            time.sleep(0.1)  

        magOffset = [0,0,0]
        magOffset[X] = (magXmin + magXmax)/2
        magOffset[Y] = (magYmin + magYmax)/2 
        magOffset[Z] = (magZmin + magZmax)/2 

        if (self.debug):
            logging.debug(("Mag Offset: x:%0.3f, y:%0.3f, z:%0.3f" % (magOffset[X], magOffset[Y], magOffset[Z])))

        #Store offset in a file
        self.storeOffsetCalibration(magOffset)

        return magOffset  

    def storeOffsetCalibration(self, magOffset): 
        msg = str(magOffset).strip('[]')       
        with open(magOffsetPath, 'w') as offsetFile:
            offsetFile.write(msg)

        logging.info("Mag offset value written properly!")
        logging.info(("Mag Offset WRITE: %s" % (msg)))

    def restoreOffsetCalibration(self):        
        if os.path.isfile(magOffsetPath):
            with open(magOffsetPath, 'r') as offsetFile:
                msg = offsetFile.read()

            msg = msg.split(",") 
            magOffset = []           
            for item in msg:
                magOffset.append(float(item))

            logging.info("Mag offset value read properly!")
            logging.info(("Mag Offset READ: x:%0.3f, y:%0.3f, z:%0.3f" % (magOffset[X], magOffset[Y], magOffset[Z])))
        else:
            logging.warning((magOffsetPath + str(" does not exist, please calibrate again!")))
            sys.exit()

        return magOffset

    def hardCalibration(self, rawMag, magOffset):
        calMag = [0,0,0]
        calMag[X] = rawMag[X] - magOffset[X]
        calMag[Y] = rawMag[Y] - magOffset[Y]
        calMag[Z] = rawMag[Z] - magOffset[Z]

        if (self.debug):
            logging.debug(("Mag Calibrated: x:%0.3f, y:%0.3f, z:%0.3f" % (calMag[X], calMag[Y], calMag[Z])))

        return calMag 

    def tiltCompensation(self, accelRollRad, accelPitchRad, rawMag):
        cosPitch = math.cos(accelPitchRad) 
        sinPitch = math.sin(accelPitchRad)
        cosRoll = math.cos(accelRollRad)
        sinRoll = math.sin(accelRollRad)

        # Tilt compensation
        compMag = [0,0,0]
        compMag[X] = (rawMag[X]*cosPitch) + (rawMag[Z]*sinPitch)
        compMag[Y] = (rawMag[X]*sinRoll*sinPitch) + (rawMag[Y]*cosRoll) - (rawMag[Z]*sinRoll*cosPitch)
        compMag[Z] = (-rawMag[X]*cosRoll*sinPitch) + (rawMag[Y]*sinRoll) - (rawMag[Z]*cosRoll*cosPitch)

        if (self.debug):
            logging.debug(("Mag Tilt: x:%0.2f, y:%0.2f" % (compMag[X], compMag[Y])))

        return compMag  

    def getHeading(self, compMagX, compMagY, decliFlag=True):
        #Only needed if the heading value does not increase when the magnetometer is rotated clockwise
        #compMagY = -compMagY;

        headingRad = math.atan2(compMagY, compMagX)

        if decliFlag == True:
            headingRad += self.declination

		# Correct for reversed heading
        if(headingRad < 0):
            headingRad += 2*math.pi
			
		# Check for wrap and compensate
        if(headingRad > 2*math.pi):
            headingRad -= 2*math.pi
	
		# Convert to degrees from radians
        headingDeg = headingRad * RAD_TO_DEG

        # Convert to degrees amd minutes
        degrees = math.floor(headingDeg)
        minutes = round(((headingDeg - degrees) * 60))

        if (self.debug):
            logging.debug(("Mag Heading: x:%0.2f" % (headingDeg)))

        self.Heading = headingDeg
        return headingDeg
     
def TestHMC5883L():
    # Enable Magnetometer and Accelerometer
    mag = GY80_Magnetometer_HMC5883L(debug=True, cal=False)
    accel = GY80_Accelerometer_ADXL345()

    LP = 0.2

    while True:
        #
        # Accelerometer
        # 
        scaledAcc = accel.scaleAcc(accel.readRawAcc())
        normAcc = accel.normalizeRawData(scaledAcc) 
        accel.calculateAngles(normAcc)

        #
        # Magnetometer
        # 

        # Magnetic declination 
        # Santo Andre @ Brazil:
        # Declination: -21 7 (Negative WEST)
        mag.setDeclination(-21,7)
        
        # Read mag value
        rawMag = mag.readRawMag()
        calMag = mag.hardCalibration(rawMag, mag.magOffset)
        scaledMag = mag.scaleMag(calMag)
        normMag = mag.normalizeRawData(scaledMag)

        print "Accel: Roll: " + str(round(accel.Roll*RAD_TO_DEG,2)) + ", Pitch: " + str(round(accel.Pitch*RAD_TO_DEG,2))
        compMag = mag.tiltCompensation(accel.Roll, accel.Pitch, normMag)   
        # Tilted        
        yaw1 = mag.getHeading(compMag[X], compMag[Y]) 
        # Not-Tilted        
        yaw2 = mag.getHeading(normMag[X], normMag[Y]) 

        time.sleep(LP)

if __name__ == '__main__':
    TestHMC5883L()
 
