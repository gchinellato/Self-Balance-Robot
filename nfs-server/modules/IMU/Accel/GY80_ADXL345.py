#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: GY80 API - Orientation sensor via I2C bus
*               ADXL345 (3-Axis Digital Accelerometer)
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                             
*************************************************
"""

from Utils.i2c.Adafruit_I2C import Adafruit_I2C
from IMU.constants import *
import math
import time
from Utils.traces.trace import *

class GY80_Accelerometer_ADXL345():
    def __init__(self, busnum=-1, debug=False):
        self.i2c = Adafruit_I2C(ADXL345_ADDRESS, busnum, debug)
        self.debug = debug
        self.Roll = None
        self.Pitch = None

        if self.i2c.readU8(ADXL345_REG_DEVID) == 0xE5:
            # Enable the accelerometer
            self.i2c.write8(ADXL345_REG_POWER_CTL, 0x08)

        logging.info("Accelerometer module initialized")

    #Read raw data
    def readRawAcc(self):
        bytes = self.i2c.readList(ADXL345_REG_DATAX0, 6)

        rawAcc = []
        #read x,y,z raw data
        for i in range(0, 6, 2):
            #merge data1|data0
            g = (bytes[i+1] << 8) | bytes[i]
            #negative values
            if g > 32767:
                g -= 65536
            rawAcc.append(g)

        if (self.debug):
            logging.debug(("Acc Raw: x:%d, y:%d, z:%d" % (rawAcc[X], rawAcc[Y], rawAcc[Z])))

        return rawAcc

    #Normalize data: v'= v/\v\ - unit vector
    def normalizeRawData(self, rawAcc):
        normAcc = [0,0,0]
        normAcc[X] = rawAcc[X]/math.sqrt(rawAcc[X]*rawAcc[X] + rawAcc[Y]*rawAcc[Y] + rawAcc[Z]*rawAcc[Z])
        normAcc[Y] = rawAcc[Y]/math.sqrt(rawAcc[X]*rawAcc[X] + rawAcc[Y]*rawAcc[Y] + rawAcc[Z]*rawAcc[Z])
        normAcc[Z] = rawAcc[Z]/math.sqrt(rawAcc[X]*rawAcc[X] + rawAcc[Y]*rawAcc[Y] + rawAcc[Z]*rawAcc[Z])
        
        if (self.debug):
            logging.debug(("Acc Normalized: x:%.3f, y:%.3f, z:%.3f" % (normAcc[X], normAcc[Y], normAcc[Z])))

        return normAcc

    #Convert value read from accelerometer to a readable value
    def scaleAcc(self, rawAcc, gForce=True):
        #Acceleration G-Force
        scaledAcc = [0,0,0]
        scaledAcc[X] = rawAcc[X] * ACC_SCALE_MULTIPLIER 
        scaledAcc[Y] = rawAcc[Y] * ACC_SCALE_MULTIPLIER
        scaledAcc[Z] = rawAcc[Z] * ACC_SCALE_MULTIPLIER

        if gForce == False:
            #Acceleration m/s2
            scaledAcc[X] = scaledAcc[X] * EARTH_GRAVITY_MS2
            scaledAcc[Y] = scaledAcc[Y] * EARTH_GRAVITY_MS2
            scaledAcc[Z] = scaledAcc[Z] * EARTH_GRAVITY_MS2

        if (self.debug):
            if gForce == True:
                logging.debug(("Acc Scaled: x:%.3f, y:%.3f, z:%.3f (g)" % (scaledAcc[X], scaledAcc[Y], scaledAcc[Z])))
            else:
                logging.debug(("Acc Scaled: x:%.3f, y:%.3f, z:%.3f (m/s2)" % (scaledAcc[X], scaledAcc[Y], scaledAcc[Z])))

        return scaledAcc

    #Calculate til anges: Pitch and roll in rad
    def calculateAngles(self, acc):
        #180 degrees - 2D 
        Roll = math.atan2(acc[Y], acc[Z])
        #Pitch = math.atan2(-acc[X], acc[Z]) 

        #90 degrees - 3D
        #Roll = math.atan2(acc[Y], math.sqrt(acc[X]*acc[X] + acc[Z]*acc[Z]))
        Pitch = math.atan2(-acc[X], math.sqrt(acc[Y]*acc[Y] + acc[Z]*acc[Z]))

        self.Roll = round(Roll, 2)
        self.Pitch = round(Pitch, 2)

        if (self.debug):
            logging.debug(("Roll:%0.2f, Pitch:%0.2f" % (self.Roll*RAD_TO_DEG, self.Pitch*RAD_TO_DEG)))
        
        return (self.Roll, self.Pitch)

def TestADXL345():
    # Enable Accelerometer
    accel = GY80_Accelerometer_ADXL345(debug=True)

    LP = 0.1

    while True:
        #
        # Accelerometer
        #

        # Read and convert raw value to gFroce and normalize values
        scaledAcc = accel.scaleAcc(accel.readRawAcc())
        normAcc = accel.normalizeRawData(scaledAcc)

        # Convert to Angle (Roll and Pitch)
        accel.calculateAngles(normAcc)

        time.sleep(LP)

if __name__ == '__main__':
    TestADXL345()
 
