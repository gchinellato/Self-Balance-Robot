#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Motion 
                DC Motor with gearbox
                Motor driver L298N
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import RPi.GPIO as GPIO
import time
import datetime
from constants import *
from Utils.traces.trace import *

class Motion():
    def __init__(self, debug=False):
        self.debug = debug

        #PID parameters
        self.lastTime = 0
        self.lastError = 0
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

        #Set up BCM GPIO numbering 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM) 

        #Set GPIO as output
        GPIO.setup(MA_FORWARD_GPIO, GPIO.OUT)
        GPIO.setup(MA_BACKWARD_GPIO, GPIO.OUT)
        GPIO.setup(MB_FORWARD_GPIO, GPIO.OUT)
        GPIO.setup(MB_BACKWARD_GPIO, GPIO.OUT)

        #Set GPIO as PWM output - 50Hz
        self.mRightForward = GPIO.PWM(MA_FORWARD_GPIO, PWM_FREQ) 
        self.mRightBackward = GPIO.PWM(MA_BACKWARD_GPIO, PWM_FREQ)
        self.mLeftForward = GPIO.PWM(MB_FORWARD_GPIO, PWM_FREQ) 
        self.mLeftBackward = GPIO.PWM(MB_BACKWARD_GPIO, PWM_FREQ)

        #Start PWM (stopped)
        self.mRightForward.start(0)
        self.mRightBackward.start(0)
        self.mLeftForward.start(0)
        self.mLeftBackward.start(0)

        logging.info("Motion module initialized") 

    def PID(self, setPoint, newValue, Kp, Ki, Kd):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        currentTime = time.time()

        #Calculate delta time
        dt = currentTime - self.lastTime

        #Calculate error
        error = setPoint - newValue

        #Calculate delta error
        de = error - self.lastError
 
        #Calculate PID terms        
        self.Cp = error #Proportional Term  
      
        self.Ci += error*dt #Integral Term
 
        #Windup guard for Integral term do not reach very large values
        if self.Ci > WINDUP_GUARD:
            self.Ci = WINDUP_GUARD
        elif self.Ci < -WINDUP_GUARD:
            self.Ci = -WINDUP_GUARD
       
        self.Cd = 0
        if dt > 0:  #no div by zero
            self.Cd = de/dt #Derivative term

        #Save for the next iteration
        self.lastError = error
        self.lastTime = currentTime        

        #Sum terms
        output = (self.Cp * Kp) + (self.Ci * Ki) + (self.Cd * Kd)

        if (self.debug):
            logging.debug(("PID output = %0.2f, newValue: %0.2f, error: %0.2f, Cp: %0.2f, Ci: %0.2f, Cd: %0.2f" % (output, newValue, error, self.Cp, self.Ci, self.Cd)))

        return output

    def motorStop(self):
        self._motorLStop()
        self._motorRStop()

    def motorMove(self, speedL, speedR):
        limitedSpeedL = self._constraint(abs(speedL))
        limitedSpeedR = self._constraint(abs(speedR))

        #limitedSpeedL += COMPENSATION
        #limitedSpeedR += COMPENSATION

        #Clockwise
        if speedL > 0:
            self.mLeftBackward.ChangeDutyCycle(0)
            self.mLeftForward.ChangeDutyCycle(limitedSpeedL)
        #Anti-Clockwise
        elif speedL < 0: 
            self.mLeftForward.ChangeDutyCycle(0)
            self.mLeftBackward.ChangeDutyCycle(limitedSpeedL)
        else:
            self._motorLStop()

        #Clockwise
        if speedR > 0:
            self.mRightBackward.ChangeDutyCycle(0)
            self.mRightForward.ChangeDutyCycle(limitedSpeedR)
        #Anti-Clockwise
        elif speedR < 0: 
            self.mRightForward.ChangeDutyCycle(0)
            self.mRightBackward.ChangeDutyCycle(limitedSpeedR)
        else:
            self._motorRStop()

        if (self.debug):
            logging.debug(("Motor speed: LEFT: %0.2f, RIGHT: %0.2f" % (speedL, speedR)))
            logging.debug(("Motor speed [LIMITED]: LEFT: %0.2f, RIGHT: %0.2f" % (limitedSpeedL, limitedSpeedR)))

    def motorShutDown(self):
        self.mRightForward.stop()
        self.mRightBackward.stop()
        self.mLeftForward.stop()
        self.mLeftBackward.stop()
        GPIO.cleanup()

    def convertRange(self, analogValue):
        #convert analog value (+1.0 ~ -1.0) to dutyCycle (100.0% ~ -100.0%)
        if not analogValue >= ANALOG_MIN and analogValue <= ANALOG_MAX:        
            logging.warning("Value out of the range (Max:1.0, Min:-1.0)")
            if analogValue > ANALOG_MAX:
                analogValue = ANALOG_MAX
            elif analogValue < ANALOG_MIN:
                analogValue = ANALOG_MIN
        return analogValue * 100.0 

    def _constraint(self, value, upperLimit=PWM_MAX, lowerLimit=PWM_MIN):        
        #Limitation of the motor velocity
        #Ensure the data is into an acceptable range
        if value > upperLimit:
            value = upperLimit
        elif value < lowerLimit:
            value = lowerLimit

        return value 

    def _motorLStop(self):
        self.mLeftForward.ChangeDutyCycle(0)
        self.mLeftBackward.ChangeDutyCycle(0)

    def _motorRStop(self):
        self.mRightForward.ChangeDutyCycle(0)
        self.mRightBackward.ChangeDutyCycle(0)   

