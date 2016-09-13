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

        self.wheelVelocityAvg = 0
        self.wheelVelocityA = 0
        self.wheelVelocityB = 0
        self.lastWheelPositionA = 0
        self.lastWheelPositionB = 0

        #Start motor PWM
        self.motorStart()

        logging.info("Motion Module initialized") 

    def updateWheelVelocity(self, dt=1.0):
        '''wheelPosition = self.getWheelPosition()
        #velocity: derivative of position (Pf - Pi)/dt
        if (dt > 0):
            self.wheelVelocity = (wheelPosition - self.lastWheelPosition)/dt
        self.lastWheelPosition = wheelPosition'''

        wheelDistanceA, wheelDistanceB = self.getDistance()
        #velocity: derivative of position (Pf - Pi)/dt
        if (dt > 0):
            self.wheelVelocityA = (wheelDistanceA - self.lastWheelPositionA)/dt
            self.wheelVelocityB = (wheelDistanceB - self.lastWheelPositionB)/dt
        self.lastWheelPositionA = wheelDistanceA
        self.lastWheelPositionB = wheelDistanceB

        self.wheelVelocityAvg = (self.wheelVelocityA+self.wheelVelocityB)/2

        if (self.debug & MODULE_MOTION):
            logging.debug(("wheelVelocity Average: %0.2f, A: %0.2f, B: %0.2f" % (self.wheelVelocityAvg, self.wheelVelocityA, self.wheelVelocityB)))

    def getWheelVelocity(self):
        if (self.debug & MODULE_MOTION):
            logging.debug(("wheelVelocity : [%0.2f cm/s] [%0.2f m/s] [%0.2f km/h]" % (self.wheelVelocityAvg, self.wheelVelocityAvg/100, (self.wheelVelocityAvg/100)*3.6)))
        return self.wheelVelocityAvg

    def getDistance(self):
        '''Get distance from each motor, should be similar'''
        return (self._encoderA.getDistance(), self._encoderB.getDistance())
        #return self._encoderA.getDistance()

    def getWheelPosition(self):
        #return self._encoderA.getTicks() + self._encoderB.getTicks()
        return (self._encoderA.getTicks(), self._encoderA.getTicks())

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

def TestMotion():
    try:
        setVerbosity("debug")
        motion = Motion() 

        LP = 0.1

        while True:
            v = float((input("Inser PWM duty cycle: ")))
            motion.motorMove(v, v)            
            for i in range(25):
                motion.updateWheelVelocity(dt=LP)
                velo = motion.getWheelVelocity()
                print("wheelVelocity Average: %0.2f, A: %0.2f, B: %0.2f" % (motion.wheelVelocityAvg, motion.wheelVelocityA, motion.wheelVelocityB))
                print ("wheelVelocity : [%0.2f cm/s] [%0.2f m/s] [%0.2f km/h]" % (velo, velo/100, (velo/100)*3.6))
                print "Distance: " + str(motion.getDistance())
                time.sleep(LP)

    except KeyboardInterrupt:
        print "Shutdown motor"
        motion.motorShutDown()

if __name__ == '__main__':
    TestMotion()
