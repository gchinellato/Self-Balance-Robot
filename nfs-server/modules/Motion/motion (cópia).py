#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Motion 
                DC Motor with gearbox/encoder
                Motor driver VNH2SP30
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import RPi.GPIO as GPIO
import time
from constants import *
from Utils.gpio_mapping import *
from Utils.traces.trace import *

#Global variables encoders count
countEncoderA = 0
countEncoderB = 0
lastInterruptTime = 0

class Motion():
    def __init__(self, debug=0):
        self.debug = debug

        #PID parameters
        self.lastTime = 0
        self.lastError = 0
        self.wheelPosition = 0
        self.lastWheelPosition = 0
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

        #Set up BCM GPIO numbering 
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM) 

        #Set GPIO as output
        GPIO.setup(MA_PWM_GPIO, GPIO.OUT) 
        GPIO.setup(MB_PWM_GPIO, GPIO.OUT)
        GPIO.setup(MA_CLOCKWISE_GPIO, GPIO.OUT)
        GPIO.setup(MA_ANTICLOCKWISE_GPIO, GPIO.OUT)
        GPIO.setup(MB_CLOCKWISE_GPIO, GPIO.OUT)
        GPIO.setup(MB_ANTICLOCKWISE_GPIO, GPIO.OUT)

        GPIO.output(MA_CLOCKWISE_GPIO, False)
        GPIO.output(MA_ANTICLOCKWISE_GPIO, False)
        GPIO.output(MB_CLOCKWISE_GPIO, False)
        GPIO.output(MB_ANTICLOCKWISE_GPIO, False)

        #Set GPIO as PWM output
        self._maPWM = GPIO.PWM(MA_PWM_GPIO, PWM_FREQ) 
        self._mbPWM = GPIO.PWM(MB_PWM_GPIO, PWM_FREQ)

        #Start PWM (stopped)
        self._maPWM.start(0)
        self._mbPWM.start(0)

        #Set GIPO as input
        GPIO.setup(MA_ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
        GPIO.setup(MA_ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(MB_ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
        GPIO.setup(MB_ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        #Set GPIO as interrupt inputs with callback functions
        #GPIO.add_event_detect(MA_ENCODER_1, GPIO.FALLING, callback=self._encoderA, bouncetime=100)
        #GPIO.add_event_detect(MB_ENCODER_2, GPIO.FALLING, callback=self._encoderB, bouncetime=100)

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

        #Proportional Term          
        self.Cp = error
      
        #Integral Term
        self.Ci += error*dt
 
        #Windup guard for Integral term do not reach very large values
        if self.Ci > WINDUP_GUARD:
            self.Ci = WINDUP_GUARD
        elif self.Ci < -WINDUP_GUARD:
            self.Ci = -WINDUP_GUARD
       
        self.Cd = 0
        #to avoid division by zero
        if dt > 0:
            #Derivative term
            self.Cd = de/dt 

        #Save for the next iteration
        self.lastError = error
        self.lastTime = currentTime        

        #Sum terms
        output = (self.Cp * Kp) + (self.Ci * Ki) + (self.Cd * Kd)

        if (self.debug & MODULE_MOTION): 
            logging.debug(("PID output = %0.2f, newValue: %0.2f, error: %0.2f, Cp: %0.2f, Ci: %0.2f, Cd: %0.2f" % (output, newValue, error, self.Cp, self.Ci, self.Cd)))

        return output

    def motorStop(self):
        self.motorMove(0, 0)

    def motorMove(self, speedA, speedB):
        limitedSpeedA = self._constraint(abs(speedA))
        limitedSpeedB = self._constraint(abs(speedB))

        limitedSpeedA += COMPENSATION
        #limitedSpeedB += COMPENSATION

        #Clockwise
        if speedA > 0:
            self._motorA(direction="CW", pwm=abs(limitedSpeedA))
        #Anti-Clockwise
        elif speedA < 0: 
            self._motorA(direction="CCW", pwm=abs(limitedSpeedA))
        #Stop        
        else:
            self._motorA()

        #Clockwise
        if speedB > 0:
            self._motorB(direction="CW", pwm=abs(limitedSpeedB))
        #Anti-Clockwise
        elif speedB < 0: 
            self._motorB(direction="CCW", pwm=abs(limitedSpeedB))
        #Stop        
        else:
            self._motorB() 

        if (self.debug & MODULE_MOTION):
            #logging.debug(("Motor speed: A: %0.2f, B: %0.2f" % (speedA, speedB)))
            logging.debug(("Motor speed [LIMITED]: A: %0.2f, B: %0.2f" % (limitedSpeedA, limitedSpeedB)))

    def motorShutDown(self):
        self.motorStop()
        self._maPWM.stop()
        self._mbPWM.stop()
        GPIO.cleanup()

    def _motorA(self, direction="", pwm=0):
        if direction == "CW":
            GPIO.output(MA_CLOCKWISE_GPIO, True)
            GPIO.output(MA_ANTICLOCKWISE_GPIO, False)
        elif direction == "CCW":
            GPIO.output(MA_CLOCKWISE_GPIO, False)
            GPIO.output(MA_ANTICLOCKWISE_GPIO, True)
        else:
            GPIO.output(MA_CLOCKWISE_GPIO, False)
            GPIO.output(MA_ANTICLOCKWISE_GPIO, False)           
        self._maPWM.ChangeDutyCycle(pwm)

    def _motorB(self, direction="", pwm=0):
        if direction == "CW":
            GPIO.output(MB_CLOCKWISE_GPIO, True)
            GPIO.output(MB_ANTICLOCKWISE_GPIO, False)
        elif direction == "CCW":
            GPIO.output(MB_CLOCKWISE_GPIO, False)
            GPIO.output(MB_ANTICLOCKWISE_GPIO, True)
        else:
            GPIO.output(MB_CLOCKWISE_GPIO, False)
            GPIO.output(MB_ANTICLOCKWISE_GPIO, False)        
        self._mbPWM.ChangeDutyCycle(pwm)

        def _encoderA(self, MA_ENCODER_1):  
            print "Interrupt Motor A"
            global countEncoderA

            #when the callback function is called due interrup on MA_ENCODER_1 and MA_ENCODER_2 is true, then is clockwise, if not counter clockwise
            if (GPIO.input(MA_ENCODER_2) == True):
                countEncoderA += 1
            else:
                countEncoderA -= 1

        def _encoderB(self, MB_ENCODER_1):  
            print "Interrupt Motor B"
            global countEncoderB

            #when the callback function is called due interrup on MB_ENCODER_1 and MB_ENCODER_2 is true, then is clockwise, if not counter clockwise
            if (GPIO.input(MB_ENCODER_2) == True):
                countEncoderB += 1
            else:
                countEncoderB -= 1

    def readEncoderA(self):
        global countEncoderA
        return countEncoderA

    def readEncoderB(self):
        global countEncoderB
        return countEncoderB

    def getWheelsPosition(self):
        global countEncoderA
        global countEncoderB
        return (countEncoderA + countEncoderB) / 2

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
