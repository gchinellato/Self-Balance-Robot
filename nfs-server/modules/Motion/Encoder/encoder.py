#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Encoder module
                DC Motor with gearbox/encoder
                Motor driver VNH2SP30
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import RPi.GPIO as GPIO
import math
import time
from Motion.constants import *
from Utils.gpio_mapping import *
from Utils.traces.trace import *

class Encoder():
    def __init__(self, name, pinEncoder1, pinEncoder2, ticksPerTurn, radius, debug=0):
        self.debug = debug
        self.name = name
        self.pinEncoder1 = pinEncoder1
        self.pinEncoder2 = pinEncoder2
        self.ticksPerTurn = ticksPerTurn
        self.radius = radius
        self.cmPerTick = 0.0
        self.ticks = 0

        #Set up BCM GPIO numbering 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM) 

        #Set GIPO as input
        GPIO.setup(pinEncoder1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
        GPIO.setup(pinEncoder2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.setCmPerTick(self.getTicksPerTurn(), self.getRadius()) 

        #Set GPIO as interrupt inputs with callback functions
        GPIO.add_event_detect(pinEncoder1, GPIO.FALLING, self.eventCallback)

        logging.info(("Encoder " + str(name) + " module initialized"))     

    def getTicks(self):
        return self.ticks

    def resetTicks(self):
        self.ticks = 0
        
        if (self.debug & MODULE_MOTION_ENCODER): 
            logging.debug(("Encoder %s ticks reseted to zero: %d" % (self.name, self.ticks)))

    def getTicksPerTurn(self):
        return self.ticksPerTurn

    def setTicksPerTurn(self, ticks):
        '''Set the number of tick to complete one turn'''
        self.ticksPerTurn = ticks

        if (self.debug & MODULE_MOTION_ENCODER): 
            logging.debug(("Encoder %s ticksPerTurn set to: %d ticks" % (self.name, self.ticksPerTurn)))

    def getRadius(self):
        '''Get radius of the wheel in cm'''
        return self.radius

    def setRadius(self, radius):
        ''' Set radius of the wheels in cm'''
        self.radius = radius

        if (self.debug & MODULE_MOTION_ENCODER): 
            logging.debug(("Encoder %s radius set to: %d cm" % (self.name, self.radius)))

    def setCmPerTick(self, ticksPerTurn, radius):
        '''Set the the lenght of the wheel, C=2*pi*radius/ticksPerTurn''' 
        if (ticksPerTurn > 0):
            self.cmPerTick = (2*math.pi*radius)/ticksPerTurn

        if (self.debug & MODULE_MOTION_ENCODER): 
            logging.debug(("Encoder %s cmPerTick is %0.2f" % (self.name, self.cmPerTick)))

    def getDistance(self):
        '''Get the distance (cm) from the stacionary position, checking the current ticks multiplied by one complete wheel in tickes'''
        return self.ticks * self.cmPerTick

    def eventCallback(self, channel):
        '''when the callback function is called due an interrup event on pinEncoder1 and pinEncoder2 is true, then is clockwise, if not it is counter-clockwise'''
        if (GPIO.input(self.pinEncoder2) == True):
            self.ticks += 1
        else:
            self.ticks -= 1

def TestEncoder():
    try:
        setVerbosity("debug") 
        encoderA = Encoder("Left", MA_ENCODER_1, MA_ENCODER_2, TICKS_PER_TURN, WHEEL_RADIUS, MODULE_MOTION_ENCODER)
        encoderB = Encoder("Right", MB_ENCODER_1, MB_ENCODER_2, TICKS_PER_TURN, WHEEL_RADIUS, MODULE_MOTION_ENCODER)

        LP = 0.1

        print "ResetTicks"
        encoderA.resetTicks()
        encoderB.resetTicks()

        print "setTicksPerTurn"
        encoderA.setTicksPerTurn(TICKS_PER_TURN)
        print "getTicksPerTurn: " + str(encoderA.getTicksPerTurn())
        print "setRadius"
        encoderA.setRadius(WHEEL_RADIUS)
        print "getRadius: " + str(encoderA.getRadius())

        print "setCmPerTick"
        encoderA.setCmPerTick(encoderA.getTicksPerTurn(), encoderA.getRadius()) 
        encoderB.setCmPerTick(encoderA.getTicksPerTurn(), encoderA.getRadius())

        while True:
            print "getTicks A: " + str(encoderA.getTicks())
            print "getDistance A: " + str(encoderA.getDistance()) + " cm"
            print "getTicks B: " + str(encoderB.getTicks())
            print "getDistance B: " + str(encoderB.getDistance()) + " cm"
            print " "
            time.sleep(1)

    except KeyboardInterrupt:
        print "Exiting"

if __name__ == '__main__':
    TestEncoder()
