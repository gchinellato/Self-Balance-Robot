#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: Pan Tilt - Micro Servo motors API with RPI.GPIO
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

        #Absolute and relatives angles
        self.angleV = 0.0
        self.angleH = 0.0
        self.scaledAngleV = 0.0
        self.scaledAngleH = 0.0

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
         12.5      2.5         180'''

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
                            pwmVertical = self.convertTo(event[0], ANALOG_MAX, ANALOG_MIN, VERTICAL_MAX, VERTICAL_MIN) 
                            self.angleV = self.convertTo(pwmVertical, POS_MAX, POS_MIN, ANGLE_MAX, ANGLE_MIN)
                            self.scaledAngleV = self.convertTo(pwmVertical, VERTICAL_MAX, VERTICAL_MIN, ANGLE_MAX, ANGLE_MIN)                           
                            self._changeV(pwmVertical)

                            if (self.debug):
                                logging.debug("PWM Vertical: " + str(pwmVertical) + "%")
                                logging.debug("Angle Vertical: " + str(self.angleV) + "deg")
                                logging.debug("Angle Scaled Vertical: " + str(self.scaledAngleV) + "deg")   

                        if event[1] != None:
                            pwmHorizontal = self.convertTo(event[1], ANALOG_MAX, ANALOG_MIN, HORIZONTAL_MAX, HORIZONTAL_MIN) 
                            self.angleH = self.convertTo(pwmHorizontal, POS_MAX, POS_MIN, ANGLE_MAX, ANGLE_MIN)
                            self.scaledAngleH = self.convertTo(pwmHorizontal, HORIZONTAL_MAX, HORIZONTAL_MIN, ANGLE_MAX, ANGLE_MIN)                                           
                            self._changeH(pwmHorizontal)

                            if (self.debug):
                                logging.debug("PWM Horizontal: " + str(pwmHorizontal) + "%")
                                logging.debug("Angle Horizontal: " + str(self.angleH) + "deg")
                                logging.debug("Angle Scaled Horizontal: " + str(self.scaledAngleH) + "deg")

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

    # As Raspberry does not have hardware PWM pins, it is used a software one, then it is necessary to do a workaround.
    def pause(self):
        self.status = 0
        self._changeV(0)
        self._changeH(0)

    def resume(self): 
        self.status = 1 

    def getAbsoluteAngles(self):
        #Get absolute angle - real angle
        return self.angleV, self.angleH

    def getScaledAngles(self):
        #Get angle relative the limits
        return self.scaledAngleV, self.scaledAngleH

    def convertTo(self, value, fromMax, fromMin, toMax, toMin):
        if not value >= fromMin and value <= fromMax:        
            logging.warning("Value out of the range (Max:"+str(fromMax)+" , Min:"+str(fromMin)+")")
            if value > fromMax:
                value = fromMax
            elif value < fromMin:
                value = fromMin

        factor = (value-fromMin)/(fromMax-fromMin)
        return factor*(toMax-toMin)+toMin 

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

