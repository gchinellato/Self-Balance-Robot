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
        self.relativeAngleV = 0.0
        self.relativeAngleH = 0.0 
        self.absoluteAngleV = 0.0
        self.absoluteAngleH = 0.0 

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
                            headV = self.convertRange(event[0])
                            dutyCycleVertical = self._constraint(headV, VERTICAL_MAX, VERTICAL_MIN) 
                            self.absoluteAngleV = self.convertDutyCycleToDegree(dutyCycleVertical)
                            self.relativeAngleV = self.convertDutyCycleToDegree(dutyCycleVertical,upperLimit=VERTICAL_MAX,lowerLimit=VERTICAL_MIN)                            
                            self._changeV(dutyCycleVertical)

                            if (self.debug):
                                logging.debug("PWM Vertical: " + str(dutyCycleVertical))
                                logging.debug("Relative Angle Vertical: " + str(self.relativeAngleV))
                                logging.debug("Absolute Angle Vertical: " + str(self.absoluteAngleV))

                        if event[1] != None:
                            headH = self.convertRange(event[1])  
                            dutyCycleHorizontal = self._constraint(headH, HORIZONTAL_MAX, HORIZONTAL_MIN) 
                            self.absoluteAngleH = self.convertDutyCycleToDegree(dutyCycleHorizontal)
                            self.relativeAngleH = self.convertDutyCycleToDegree(dutyCycleHorizontal,upperLimit=HORIZONTAL_MAX,lowerLimit=HORIZONTAL_MIN)                                               
                            self._changeH(dutyCycleHorizontal)

                            if (self.debug):
                                logging.debug("PWM Horizontal: " + str(dutyCycleHorizontal)) 
                                logging.debug("Angle Horizontal: " + str(self.relativeAngleH))
                                logging.debug("Absolute Angle Vertical: " + str(self.absoluteAngleH))

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

    def getRelativeAngles(self):
        #Get angles relative the upper and lower limits (upperLimit and lowerLimit)
        return self.relativeAngleV, self.relativeAngleH

    def getAbsoluteAngles(self):
        #Get angles relative the max and min positions (POS_MAX and POS_MIN)
        return self.absoluteAngleV, self.absoluteAngleH
 
    def convertDutyCycleToDegree(self, dutyCycle, upperLimit=POS_MAX, lowerLimit=POS_MIN):
        #Convert duty cycle value to degree according the boundary
        return (dutyCycle-lowerLimit)/((upperLimit-lowerLimit)/(ANGLE_MAX-ANGLE_MIN))

    def convertDegreeToDutyCycle(self, degree, upperLimit=POS_MAX, lowerLimit=POS_MIN):
        #Convert degree value to duty cycle according the boundary
        return ((upperLimit-lowerLimit)/(ANGLE_MAX-ANGLE_MIN))*degree + lowerLimit

    def convertDutyCycleToAnalogValue(self, dutyCycle, upperLimit=POS_MAX, lowerLimit=POS_MIN):
        #Convert duty cycle value to analog value according the boundary
        # (duty cycle - midLimit) / (upperLimit - midLimit)
        return (dutyCycle-(upperLimit-(upperLimit-lowerLimit)/2))/(upperLimit-(upperLimit-(upperLimit-lowerLimit)/2))

    def convertDegreeToAnalogValue(self, degree, upperLimit=ANGLE_MAX, lowerLimit=ANGLE_MIN):
        #Convert degree to analog value according the boundary
        # (degree - midLimit) / (upperLimit - midLimit)
        return (degree-(upperLimit-(upperLimit-lowerLimit)/2))/(upperLimit-(upperLimit-(upperLimit-lowerLimit)/2))

    def convertRange(self, analogValue):
        #Convert analog value (+1.0 ~ -1.0) to dutyCycle (12.5% ~ 2.5%)
        if not analogValue >= ANALOG_MIN and analogValue <= ANALOG_MAX:        
            logging.warning("Value out of the range (Max:1.0, Min:-1.0)")
            if analogValue > ANALOG_MAX:
                analogValue = ANALOG_MAX
            elif analogValue < ANALOG_MIN:
                analogValue = ANALOG_MIN
        return (analogValue*(POS_MAX-POS_NEUTRAL))+POS_NEUTRAL  

    def _constraint(self, value, upperLimit=POS_MAX, lowerLimit=POS_MIN): 
        #Limitation of the panTilt movement 
        #Proporcional value to reach upper and lower limits
        #For instance, value=7.5 (50%) and full range 2.5(0%) up to 12.5(100%), but it is necessary to restrict the lower part to 8.5,
        #then the new range is 8.5(0%) up to 12.5(100%), however to keep the same percentage (50%) the value should be 10.5.
        value = lowerLimit + (upperLimit-lowerLimit) * ((value-POS_MIN)/(POS_MAX-POS_MIN))

        #Ensure the data is into an acceptable range
        if value > upperLimit:
            value = upperLimit
        elif value < lowerLimit:
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

