#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: Pan Tilt - Micro Servo motors API
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import time
import os
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

        #ServoBlaster is what we use to control the servo motors
        self.servoBlaster = open('/dev/servoblaster', 'w')
        logging.debug("PWM Horizontal init: " + str((HORIZONTAL_MAX-HORIZONTAL_MIN)/2) + "%")
        self.servoBlaster.write(SERVO_H + '=' + str((HORIZONTAL_MAX-HORIZONTAL_MIN)/2) + '%' + '\n')
        self.servoBlaster.flush()
        logging.debug("PWM Vertical init: " + str(VERTICAL_MAX) + "%")
        self.servoBlaster.write(SERVO_V + '=' + str(VERTICAL_MAX) + '%' + '\n')
        self.servoBlaster.flush()
        time.sleep(0.1) 

        logging.info("Pan-Tilt Thread initialized")

    #Override method
    def run(self):
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
                    if event[0] != None:
                        pwmVertical = self.convertRange(event[0])
                        pwmVertical = self._constraint(pwmVertical, VERTICAL_MAX, VERTICAL_MIN)                           
                        self._changeV(pwmVertical)

                        if (self.debug):
                            logging.debug("PWM Vertical: " + str(pwmVertical)) + "%"

                    if event[1] != None:
                        pwmHorizontal = self.convertRange(event[1])  
                        pwmHorizontal = self._constraint(pwmHorizontal, HORIZONTAL_MAX, HORIZONTAL_MIN)                                              
                        self._changeH(pwmHorizontal)

                        if (self.debug):
                            logging.debug("PWM Horizontal: " + str(pwmHorizontal)) + "%"

            except Queue.Empty:
                if (self.debug):
                    logging.debug("Queue Empty")
                pass
            finally:
                lastTime = currentTime                
                self._lock.release()                      
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end        
        self._stopEvent.set()
        self.servoBlaster.close()
        threading.Thread.join(self, timeout=timeout) 

    def getEvent(self, timeout=1):
        return self._workQueue.get(timeout=timeout)

    def putEvent(self, event):     
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():     
            self._workQueue.put(event)

    def convertRange(self, analogValue):
        #Convert analog value (+1.0 ~ -1.0) to pwm (0% ~ 100%)
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

    def _changeV(self, percentage):
        self.servoBlaster.write(SERVO_V + '=' + str(percentage) + '%' + '\n')
        self.servoBlaster.flush()

    def _changeH(self, percentage):
        self.servoBlaster.write(SERVO_V + '=' + str(percentage) + '%' + '\n')
        self.servoBlaster.flush()

