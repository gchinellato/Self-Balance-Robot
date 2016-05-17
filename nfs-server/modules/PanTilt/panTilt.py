#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: Pan Tilt - Micro Servo motors API
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import RPi.GPIO as GPIO
import time
import threading
import Queue
from constants import *
from Utils.traces.trace import *

SERVO_V_GPIO = 23 #servo vertical move
SERVO_H_GPIO = 24 #servo horizontal move

POS_MAX = 12.5 # Max position 180 degree
POS_MIN = 2.5 # Min position 0 degree
POS_NEUTRAL = 7.5 # Default position 90 degreee (neutral)

FREQ = 50 # PWM frequency

class PanTiltThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=False):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug 

        #Queue to communicate between threads
        self._workQueue = queue
        self._lock = threading.Lock()
        
        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.0

        self.dutyCycleVertical = 0.0
        self.dutyCycleHorizontal = 0.0 
        self.angleVertical = 0.0
        self.angleHorizontal = 0.0 

        GPIO.setwarnings(False) # disable warnings
        GPIO.setmode(GPIO.BCM) # set up BCM GPIO numbering 
        GPIO.setup(SERVO_V_GPIO, GPIO.OUT) # set GPIO as output
        GPIO.setup(SERVO_H_GPIO, GPIO.OUT) # set GPIO as output

        ''' SERVO
        PERIOD = 20ms (50Hz)
         
         DT(%)    Time(ms)     Degree
         2,5       0,5         0
         5.0       1.0         45   
         7.5       1.5         90
         10.0      2.0         135
         12.5      2.5         180

        one degree = (12.5-2.5)/(180-0) = 0.05555
        DutyCycle=((POS_MAX-POS_MIN)/(180-0))*Angle + POS_MIN
        Angle = (DutyCycle - POS_MIN) / ((POS_MAX-POS_MIN)/(180-0))'''        

        #PWM output for f=50Hz / t=20ms 
        self.pwmV = GPIO.PWM(SERVO_V_GPIO, FREQ) 
        self.pwmH = GPIO.PWM(SERVO_H_GPIO, FREQ)
        self.status = 0

        logging.info("Pan-Tilt Thread initialized")

    #Override method
    def run(self):
        self._startPWM(0, 0)
        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                self._lock.acquire()

                currentTime = time.time()

                #Calculate time since the last time it was called
                #if (self.debug):
                #    logging.debug("Duration: " + str(currentTime - lastTime))
                
                event = self.getEvent()
                if event != None:
                    if self.status == 1:
                        if event[0] != None:
                            #Ensure the data is into an acceptable range: 9.5% ~ 12.5% / 126deg ~ 180deg
                            self.angleVertical = event[0]
                            self.dutyCycleVertical = self._constraint(event[0], 12.5, 9.5)                            
                            self._changeV(self.dutyCycleVertical)

                            if (self.debug):
                                logging.debug("PWM Vertical: " + str(self.dutyCycleVertical))
                                logging.debug("Angle Vertical: " + str(self.angleVertical))

                        if event[1] != None:
                            #Ensure the data is into an acceptable range: 4.5% ~ 8.5% / 36deg ~ 108deg
                            self.angleHorizontal = event[1] 
                            self.dutyCycleHorizontal = self._constraint(event[1], 8.5, 4.5)                                               
                            self._changeH(self.dutyCycleHorizontal)

                            if (self.debug):
                                logging.debug("PWM Horizontal: " + str(self.dutyCycleHorizontal)) 
                                logging.debug("Angle Horizontal: " + str(self.angleHorizontal))

            except Queue.Empty:
                if (self.debug):
                    logging.debug("Queue Empty")
                self.pause()
                pass
            finally:
                lastTime = currentTime                
                self._lock.release()                      
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end        
        self._stopEvent.set()
        self._stopPWM()
        threading.Thread.join(self, timeout=timeout) 

    def getEvent(self, timeout=1):
        return self._workQueue.get(timeout=timeout)

    def putEvent(self, event):     
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():       
            self._workQueue.put(event)

    # As Raspberry does not have hardware PWM pins, it is used a software one, then it is necessary do a workaround.
    def pause(self):
        self.status = 0
        self._changeV(0)
        self._changeH(0)

    def resume(self): 
        self.status = 1 

    def getAngles(self):
        return self.angleVertical, self.angleHorizontal
    
    def getDutyCycles(self):
        return self.dutyCycleVertical, self.dutyCycleHorizontal

    def convertDutyCycleInDegree(self, dutyCycle):
        return (dutyCycle-POS_MIN)/((POS_MAX-POS_MIN)/(180-0))

    def convertDegreeInDutyCycle(self, degree):
        return ((POS_MAX-POS_MIN)/(180-0))*degree + POS_MIN

    def convertDegreeInAnalogValue(self, degree):
        return ((1.0-(-1.0))/(180-0))*degree + (-1.0)

    def convertRange(self, value):
        #convert analog value (+1.0 ~ -1.0) to dutyCycle (12.5% ~ 2.5%)
        if value <= 1.0 and value >= -1.0:
            if value > 0:
                value = (value*(POS_MAX-POS_NEUTRAL))+POS_NEUTRAL
            elif value < 0:
                value = (value*(POS_MIN-POS_NEUTRAL))-POS_NEUTRAL
            else:
                value = POS_NEUTRAL
        else:
            value = POS_NEUTRAL
            logging.warning("Value out of the range (Max:1.0, Min:-1.0)")

        return abs(value)  

    def _constraint(self, value, upperLimit=POS_MAX, lowerLimit=POS_MIN): 
        #Proporcional value to reach upper and lower limit      
        value = lowerLimit + (upperLimit-lowerLimit)*((value-POS_MIN)/(POS_MAX-POS_MIN))

        #Ensure the data is into an acceptable range
        if value > upperLimit:
            value = upperLimit
        if value < lowerLimit:
            value = lowerLimit

        return value

    def _startPWM(self, dutyCycleV, dutyCycleH):
        self.pwmV.start(dutyCycleV)
        self.pwmH.start(dutyCycleH)
        self.status = 1

    def _stopPWM(self):
        self.pwmV.stop()
        self.pwmH.stop()
        self.status = 0

    def _changeV(self, dutyCycleV):
        self.pwmV.ChangeDutyCycle(dutyCycleV)

    def _changeH(self, dutyCycleH):
        self.pwmH.ChangeDutyCycle(dutyCycleH)

