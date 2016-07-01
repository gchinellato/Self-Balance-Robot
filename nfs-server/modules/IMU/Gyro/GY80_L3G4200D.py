#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: GY80 API - Orientation sensor via I2C bus
*               L3G4200D (3-Axis Angular Rate Sensor / Gyro)
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

from Utils.i2c.Adafruit_I2C import Adafruit_I2C
from IMU.constants import *
import math
import time
from Utils.traces.trace import *

class GY80_Gyro_L3G42000D():
    def __init__(self, busnum=-1, debug=0):
        self.i2c = Adafruit_I2C(L3G42000D_ADDRESS, busnum, debug)
        self.debug = debug

        self.rateGyro = None

        #Enable gyro (Normal mode - X,Y,Z axes)
        self.i2c.write8(L3G42000D_CTRL_REG1, 0x0F) 
        #Full scale selection (2000 dps)
        self.i2c.write8(L3G42000D_CTRL_REG4, 0x30) 

        logging.info("Gyroscope module initialized")

    def getSignedNumber(self, number):
        if number & (1 << 15):
            return number | ~65535
        else:
            return number & 65535

    #Read raw data
    def readRawGyro(self):
        #read x,y,z raw data and merge MSB|LSB
        X_L = self.i2c.readU8(L3G42000D_OUT_X_L)
        X_H = self.i2c.readU8(L3G42000D_OUT_X_H)
        x = X_H << 8 | X_L
        Y_L = self.i2c.readU8(L3G42000D_OUT_Y_L)
        Y_H = self.i2c.readU8(L3G42000D_OUT_Y_H)
        y = Y_H << 8 | Y_L
        Z_L = self.i2c.readU8(L3G42000D_OUT_Z_L)
        Z_H = self.i2c.readU8(L3G42000D_OUT_Z_H)
        z = Z_H << 8 | Z_L

        rawGyro = []
        x = self.getSignedNumber(x)
        rawGyro.append(x)
        y = self.getSignedNumber(y)
        rawGyro.append(y)
        z = self.getSignedNumber(z)
        rawGyro.append(z)

        if (self.debug & MODULE_IMU_GYRO):
            logging.debug(("Gyro Raw: x:%d, y:%d, z:%d" % (rawGyro[X], rawGyro[Y], rawGyro[Z])))

        return rawGyro

    #Convert raw gyro to deg/s (dps)
    def scaleGyro(self, rawGyro):
        rateGyro = [0,0,0]
        rateGyro[X] = rawGyro[X] * GYRO_SENSITIVITY
        rateGyro[Y] = rawGyro[Y] * GYRO_SENSITIVITY
        rateGyro[Z] = rawGyro[Z] * GYRO_SENSITIVITY

        if (self.debug & MODULE_IMU_GYRO):
            logging.debug(("Gyro Rate: x:%d, y:%d, z:%d (deg/s)" % (rateGyro[X], rateGyro[Y], rateGyro[Z])))

        self.rateGyro = rateGyro
        return rateGyro

    def calculateAngles(self, rateGyro, DT):
        angleGyro = [0,0,0]
        angleGyro[X] += rateGyro[X] * DT
        angleGyro[Y] += rateGyro[Y] * DT
        angleGyro[Z] += rateGyro[Z] * DT  

        if (self.debug & MODULE_IMU_GYRO):
            logging.debug(("Gyro Angle: x:%0.2f, y:%0.2f, z:%0.2f" % (angleGyro[X], angleGyro[Y], angleGyro[Z])))

        return angleGyro      

def TestL2G42000D():
    # Enable Gyro
    gyro = GY80_Gyro_L3G42000D(debug=True)
        
    LP = 0.1

    while True:
        #
        # Gyroscope
        #

        #Read gyro raw values
        rawGyro = gyro.readRawGyro()
        rateGyro = gyro.scaleGyro(rawGyro)

        # Convert to Angle (X,Y,Z)
        gyro.calculateAngles(rateGyro, LP) 

        time.sleep(LP)    

if __name__ == '__main__':
    TestL2G42000D()

