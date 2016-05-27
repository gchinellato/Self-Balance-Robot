#!/usr/bin/python

import os
import time
import multiprocessing
import Queue
import logging
import pygame
from Utils.traces.trace import *
from Comm.Bluetooth.controller_ps3 import PS3_ControllerThread
from PanTilt.panTilt import PanTiltThread

'''Servo mapping:
     0 on P1-7           GPIO-4
     1 on P1-11          GPIO-17
     2 on P1-12          GPIO-18
     3 on P1-13          GPIO-27
     4 on P1-15          GPIO-22
     5 on P1-16          GPIO-23
     6 on P1-18          GPIO-24
     7 on P1-22          GPIO-25'''

'''#SERVO PINS
SERVO_H = '2' #pin 12 BCM 18
SERVO_V = '5' #pin 16 BCM 23

#Position
POS_MAX = 100 # Max position 180 degree
POS_MIN = 0 # Min position 0 degree
POS_NEUTRAL = 50 # Default position 90 degreee (neutral)

#Position limits (in percentage)
HORIZONTAL_MAX = 55 
HORIZONTAL_MIN = 15

VERTICAL_MAX = 95
VERTICAL_MIN = 60

#Angle limits
ANGLE_MAX = 180.0
ANGLE_MIN = 0.0

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0

class PanTiltThread(multiprocessing.Process):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=multiprocessing.Queue(), debug=False):
        multiprocessing.Process.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug 

        #Queue to communicate between threads
        self._workQueue = queue
        
        #Event to signalize between threads
        self._stopEvent = multiprocessing.Event()
        self._sleepPeriod = 0.0

        #Absolute and relatives angles
        self.relativeAngleV = 0.0
        self.relativeAngleH = 0.0 
        self.absoluteAngleV = 0.0
        self.absoluteAngleH = 0.0

        os.system('sudo servod')
        time.sleep(0.1)

        logging.info("Pan-Tilt Thread initialized")

    #Override method
    def run(self):
        #Initial position
        self._changeV((VERTICAL_MAX-VERTICAL_MIN)/2 + VERTICAL_MIN)
        self._changeH((HORIZONTAL_MAX-HORIZONTAL_MIN)/2 + HORIZONTAL_MIN)
        time.sleep(0.1) 

        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                currentTime = time.time()
                event = self.getEvent()
                if event != None:
                    if event[0] != None:
                        pwmVertical = self.convertRange(event[0])
                        pwmVertical = self._constraint(pwmVertical, VERTICAL_MAX, VERTICAL_MIN)                           
                        self._changeV(pwmVertical)

                        if (self.debug):
                            logging.debug("PWM Vertical: " + str(pwmVertical) + "%")

                    if event[1] != None:
                        pwmHorizontal = self.convertRange(event[1])  
                        pwmHorizontal = self._constraint(pwmHorizontal, HORIZONTAL_MAX, HORIZONTAL_MIN)                                              
                        self._changeH(pwmHorizontal)

                        if (self.debug):
                            logging.debug("PWM Horizontal: " + str(pwmHorizontal) + "%")
            except Queue.Empty:
                if (self.debug):
                    logging.debug("Queue Empty")
                pass
            finally:
                lastTime = currentTime 
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end        
        self._stopEvent.set()
        os.system('sudo killall servod')
        multiprocessing.Process.join(self, timeout=timeout) 

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
        servoBlaster = open('/dev/servoblaster', 'w')
        servoBlaster.write(SERVO_V + '=' + str(percentage) + '%' + '\n')
        servoBlaster.flush()
        servoBlaster.close()

    def _changeH(self, percentage):
        servoBlaster = open('/dev/servoblaster', 'w')
        servoBlaster.write(SERVO_H + '=' + str(percentage) + '%' + '\n')
        servoBlaster.flush()
        servoBlaster.close()'''

############################################################

eventQueue = Queue.Queue()
panTiltQueue = multiprocessing.Queue()

#Joystick thread
joy = PS3_ControllerThread(name="PS3", queue=eventQueue, debug=False)
joy.daemon = True
joy.start()

#Pan-Tilt thread
panTilt = PanTiltThread(name="PanTilt", queue=panTiltQueue, debug=True)
panTilt.daemon = True
panTilt.start() 

try:
    while True:
        try:
            event = eventQueue.get(timeout=2) 
            if event != None: 
                if event[0] == "PS3" and joy.joyStatus != None:
                    if (event[1].type == pygame.JOYAXISMOTION) and (event[1].axis != joy.A_ACC_X) and (event[1].axis != joy.A_ACC_Y) and (event[1].axis != joy.A_ACC_Z):
                        if event[1].axis == joy.A_R3_V:
                            headV = event[1].value
                            panTilt.putEvent((headV, None))
                            #logging.debug(("R3 Vertical: {0}, {1}, {2}".format(event[1].axis, event[1].value, headV)))
                        if event[1].axis == joy.A_R3_H:
                            headH = -event[1].value 
                            panTilt.putEvent((None, headH))
                            #logging.debug(("R3 Horizontal: {0}, {1}, {2}".format(event[1].axis, event[1].value, headH)))
                    
                    if event[1].type == pygame.JOYBUTTONDOWN or event[1].type == pygame.JOYBUTTONUP:
                        #logging.debug(joy.parseEvent(event[1]))
                        if event[1].button == joy.B_SQR:
                            logging.debug("Button Square")
        except Queue.Empty:
            logging.debug("Queue Empty")
            pass
        finally:
            time.sleep(0.0)
finally:
    logging.info("########################################")
    logging.info("Exiting...")
    joy.join()
    panTilt.join(timeout=2)


