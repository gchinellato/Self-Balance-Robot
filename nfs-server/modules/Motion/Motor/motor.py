#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Motor module
*               DC Motor with gearbox/encoder
*               Motor driver VNH2SP30
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import RPi.GPIO as GPIO
import time
from Motion.constants import *
from Utils.gpio_mapping import *
from Utils.traces.trace import *

class Motor():
    def __init__(self, name, pinPWM, pinCW, pinCCW, debug=0):
        self.debug = debug
        self.name = name
        self.pinPWM = pinPWM
        self.pinCW = pinCW
        self.pinCCW = pinCCW

        #Set up BCM GPIO numbering 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM) 

        #Set GPIO as output
        GPIO.setup(pinPWM, GPIO.OUT) 
        GPIO.setup(pinCW, GPIO.OUT)
        GPIO.setup(pinCCW, GPIO.OUT)

        GPIO.output(pinCW, False)
        GPIO.output(pinCCW, False)

        #Set GPIO as PWM output
        self._motorPWM = GPIO.PWM(pinPWM, PWM_FREQ) 

        logging.info(("Motor " + str(name) + " module initialized"))  

    def start(self):
        '''Start PWM (stopped)'''
        self._motorPWM.start(0)

        if (self.debug & MODULE_MOTION_MOTOR):
            logging.debug(("Motor %s started" % (self.name)))

    def stop(self):
        '''Stop motor (speed to zero), it is not necessary to restart the motor'''
        self._motorPWM.ChangeDutyCycle(0)

        if (self.debug & MODULE_MOTION_MOTOR):
            logging.debug(("Motor %s stopepd" % (self.name)))

    def shutdown(self):
        '''Disable motor, it is not necessary to restart the motor'''
        self._motorPWM.stop()
        GPIO.cleanup()

        if (self.debug & MODULE_MOTION_MOTOR):
            logging.debug(("Motor %s is down" % (self.name)))

    def setSpeed(self, direction="", pwm=0):
        '''Set motor speed'''
        if direction == "CW":
            GPIO.output(self.pinCW, True)
            GPIO.output(self.pinCCW, False)
        elif direction == "CCW":
            GPIO.output(self.pinCW, False)
            GPIO.output(self.pinCCW, True)
        else:
            GPIO.output(self.pinCW, False)
            GPIO.output(self.pinCCW, False)           
        self._motorPWM.ChangeDutyCycle(pwm)

        if (self.debug & MODULE_MOTION_MOTOR):
            logging.debug(("Motor %s: Direction %s and Speed %d" % (self.name, direction, pwm)))

def TestMotor():
    try:
        setVerbosity("debug") 
        motorA = Motor("Left", MA_PWM_GPIO, MA_CLOCKWISE_GPIO, MA_ANTICLOCKWISE_GPIO, MODULE_MOTION_MOTOR) 
        motorB = Motor("Right", MB_PWM_GPIO, MB_CLOCKWISE_GPIO, MB_ANTICLOCKWISE_GPIO, MODULE_MOTION_MOTOR) 

        LP = 0.1

        print "Start motor"
        motorA.start()
        motorB.start()
        #motorA.setSpeed(direction="CW", pwm=15)
        #motorB.setSpeed(direction="CW", pwm=10)
        #time.sleep(1000)
        for i in range(100):
            print "Set speed CW: " + str(i)
            motorA.setSpeed(direction="CW", pwm=i)
            motorB.setSpeed(direction="CW", pwm=i)
            time.sleep(LP)
        for i in range(100):
            print "Set speed CCW: " + str(i)    
            motorA.setSpeed(direction="CCW", pwm=i)
            motorB.setSpeed(direction="CCW", pwm=i)
            time.sleep(LP)    

        print "Stop motor"
        motorA.setSpeed()
        motorB.setSpeed()
        motorA.stop()
        motorB.stop()

    except KeyboardInterrupt:
        print "Shutdown motor"
        motorA.shutdown()
        motorB.shutdown()

if __name__ == '__main__':
    TestMotor()

