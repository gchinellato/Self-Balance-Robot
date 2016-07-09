#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Motion 
*               DC Motor with gearbox/encoder
*               Motor driver VNH2SP30
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import time
from Motion.Motor.motor import Motor
from Motion.Encoder.encoder import Encoder
from constants import *
from Utils.gpio_mapping import *
from Utils.traces.trace import *

class Motion():
    def __init__(self, debug=0):
        self.debug = debug

        self._motorA = Motor("Left", MA_PWM_GPIO, MA_CLOCKWISE_GPIO, MA_ANTICLOCKWISE_GPIO, debug) 
        self._motorB = Motor("Right", MB_PWM_GPIO, MB_CLOCKWISE_GPIO, MB_ANTICLOCKWISE_GPIO, debug) 

        self._encoderA = Encoder("Left", MA_ENCODER_1, MA_ENCODER_2, TICKS_PER_TURN, WHEEL_RADIUS, debug)
        self._encoderB = Encoder("Right", MB_ENCODER_1, MB_ENCODER_2, TICKS_PER_TURN, WHEEL_RADIUS, debug)

        self.wheelVelocity = 0
        self.lastWheelPosition = 0

        #Start motor PWM
        self.motorStart()

        logging.info("Motion module initialized") 

    def updateWheelVelocity(self):
        wheelPosition = self._getWheelPosition()
        #velocity: derivative of position (Pf - Pi)/dt
        self.wheelVelocity = wheelPosition - self.lastWheelPosition 
        self.lastWheelPosition = wheelPosition

        if (self.debug & MODULE_MOTION):
            logging.debug(("wheelVelocity: %0.2f" % (self.wheelVelocity)))

    def _getWheelPosition(self):
        return self._encoderA.getTicks() + self._encoderB.getTicks()

    def motorMove(self, speedA, speedB):
        '''Set motor speed, checking the boundaries and adding compensations for each motor (if necessary)'''
        limitedSpeedA = self._constraint(abs(speedA))
        limitedSpeedB = self._constraint(abs(speedB))

        limitedSpeedA += COMPENSATION_A
        limitedSpeedB += COMPENSATION_B

        #Clockwise
        if speedA > 0:
            self._motorA.setSpeed(direction="CW", pwm=abs(limitedSpeedA))
        #Anti-Clockwise
        elif speedA < 0: 
            self._motorA.setSpeed(direction="CCW", pwm=abs(limitedSpeedA))
        #Stop        
        else:
            self._motorA.setSpeed()

        #Clockwise
        if speedB > 0:
            self._motorB.setSpeed(direction="CW", pwm=abs(limitedSpeedB))
        #Anti-Clockwise
        elif speedB < 0: 
            self._motorB.setSpeed(direction="CCW", pwm=abs(limitedSpeedB))
        #Stop        
        else:
            self._motorB.setSpeed() 

        if (self.debug & MODULE_MOTION):
            #logging.debug(("Motor speed: A: %0.2f, B: %0.2f" % (speedA, speedB)))
            logging.debug(("Motor speed [LIMITED]: A: %0.2f, B: %0.2f" % (limitedSpeedA, limitedSpeedB)))

    def motorStart(self):
        '''Start motor speed stopped'''
        self._motorA.start()
        self._motorB.start()

    def motorStop(self):
        '''Set motor speed to zero, it is not necessary to restart the motor'''
        self._motorA.stop()
        self._motorB.stop()

    def motorShutDown(self):
        '''Disable motor pwm, it is necessary to restart the motor by motor.start()'''
        self._motorA.shutdown()
        self._motorB.shutdown()

    def getDistance(self):
        '''Get distance from each motor, should be similar'''
        return (self._encoderA.getDistance(), self._encoderB.getDistance())

    def convertRange(self, analogValue):
        '''Convert analog value (+1.0 ~ -1.0) to dutyCycle (100.0% ~ -100.0%)'''
        if not analogValue >= ANALOG_MIN and analogValue <= ANALOG_MAX:        
            logging.warning("Value out of the range (Max:1.0, Min:-1.0)")
            if analogValue > ANALOG_MAX:
                analogValue = ANALOG_MAX
            elif analogValue < ANALOG_MIN:
                analogValue = ANALOG_MIN
        return analogValue * 100.0 

    def _constraint(self, value, upperLimit=PWM_MAX, lowerLimit=PWM_MIN):        
        '''Limitation of the motor velocity'''
        #Ensure the data is into an acceptable range
        if value > upperLimit:
            value = upperLimit
        elif value < lowerLimit:
            value = lowerLimit
        return value 
