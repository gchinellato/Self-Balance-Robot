#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: Bluetooth controller API
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import pygame
import time
import threading
import Queue
from Utils.traces.trace import *

'''Todo: This class does not support hot plug event, if disconnected once, it is necessary restast the thread'''

class PS3_ControllerThread(threading.Thread):
    #Axes
    A_L3_H = 0
    A_L3_V = 1
    A_R3_H = 2
    A_R3_V = 3
    A_ACC_X = 4
    A_ACC_Y = 5
    A_ACC_Z = 6
    A_UP = 8
    A_RIGHT = 9
    A_DOWN = 10
    A_LEFT = 11
    A_L2 = 12
    A_R2 = 13
    A_L1 = 14
    A_R1 = 15
    A_TRI = 16
    A_CIRC = 17
    A_X = 18
    A_SQR = 19

    #Buttons
    B_SELECT = 0
    B_L3 = 1
    B_R3 = 2
    B_START = 3
    B_UP = 4
    B_RIGHT = 5
    B_DOWN = 6
    B_LEFT = 7
    B_L2 = 8
    B_R2 = 9
    B_L1 = 10
    B_R1 = 11
    B_TRI = 12
    B_CIRC = 13
    B_X = 14
    B_SQR = 15
    B_PS = 16

    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=0):
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
        self._sleepPeriod = 0.01
        
        self.j = None
        self.joyStatus = None
        self.joyID = None
        self.joyName = None
        self.numAxes = None
        self.numButtons = None

        logging.info("Joystick Thread initialized")
      
    #Override method
    def run(self):        
        self.initJoy()
        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                self._lock.acquire()

                currentTime = time.time()

                #Calculate time since the last time it was called
                #if (self.debug & MODULE_BLUETOOTH):
                #    logging.debug("Duration: " + str(currentTime - lastTime))

                if self.joyStatus != None:                
                    # Check for any queued events and then processes each one 
                    events = pygame.event.get()
                    for event in events:
                        #self.putEvent(self.name, event)
                        #Do not send Accelerometer events to reduce overhead
                        if (event.type == pygame.JOYAXISMOTION) and (event.axis != self.A_ACC_X) and (event.axis != self.A_ACC_Y) and (event.axis != self.A_ACC_Z):                                     
                            self.putEvent(self.name, event)
                            #logging.debug(event)
                        if (event.type == pygame.JOYBUTTONDOWN) or (event.type == pygame.JOYBUTTONUP):
                            self.putEvent(self.name, event)
                            #logging.debug(event)
                        i = 1
                else:
                    #Try to connect in the Bluetooth controller again
                    time.sleep(5)
                    self.initJoy()
            except Queue.Full:
                if (self.debug & MODULE_BLUETOOTH):
                    logging.debug("Queue Full")
                pass
            finally:
                lastTime = currentTime
                self._lock.release()           
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end       
        self._stopEvent.set()
        pygame.quit()
        threading.Thread.join(self, timeout=timeout)  

    def getEvent(self, timeout=2):
        return self._workQueue.get(timeout=timeout)

    def putEvent(self, name, event):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():       
            self._workQueue.put((name, event))

    def initJoy(self):
        pygame.quit()
        pygame.init() 
        if pygame.joystick.get_count() != 0:
            self.j = pygame.joystick.Joystick(pygame.joystick.get_count()-1)
            self.j.init()
            self.getInfo(1)
            logging.info("Joystick detected")
        else:
            self.getInfo(0)
            logging.info("Joystick not found")            

    def getInfo(self, status):
        if status == 1:
            self.joyStatus = self.j.get_init()
            self.joyID = self.j.get_id()
            self.joyName = self.j.get_name()
            self.numAxes = self.j.get_numaxes()
            self.numButtons = self.j.get_numbuttons()
        
            logging.info("Joystick Information")
            logging.info("Joystick Status: {0}".format(self.joyStatus))
            logging.info("Joystick ID: {0}".format(self.joyID))
            logging.info("Joystick Name: {0}".format(self.joyName))
            logging.info("Joystick Axes: {0}".format(self.numAxes))
            logging.info("Joystick Buttons: {0}".format(self.numButtons))
        else:
            self.joyStatus = None
            self.joyID = None
            self.joyName = None
            self.numAxes = None
            self.numButtons = None

    def parseEvent(self, event):
        string = ""
        if event.type == pygame.JOYAXISMOTION:
            if event.axis == self.A_L3_H:
                string = "Analog L3 Horizonal"
            if event.axis == self.A_L3_V:
                string = "Analog L3 Vertical"
            if event.axis == self.A_R3_H:
                string = "Analog R3 Horizonal"
            if event.axis == self.A_R3_V:
                string = "Analog R3 Vertical"
            if event.axis == self.A_ACC_X:
                string = "Accelerometer X"
            if event.axis == self.A_ACC_Y:
                string = "Accelerometer Y"
            if event.axis == self.A_ACC_Z:
                string = "Accelerometer Z"
            if event.axis == self.A_UP:
                string = "Analog UP"
            if event.axis == self.A_RIGHT:
                string = "Analog RIGHT"
            if event.axis == self.A_DOWN:
                string = "Analog DOWN"
            if event.axis == self.A_LEFT:
                string = "Analog LEFT"
            if event.axis == self.A_L2:
                string = "Analog L2"
            if event.axis == self.A_R2:
                string = "Analog R2"
            if event.axis == self.A_L1:
                string = "Analog L1"
            if event.axis == self.A_R1:
                string = "Analog R1"
            if event.axis == self.A_TRI:
                string = "Analog Triangle"
            if event.axis == self.A_CIRC:
                string = "Analog Circle"
            if event.axis == self.A_X:
                string = "Analog Xis"
            if event.axis == self.A_SQR:
                string = "Analog Square"

        if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
            if event.button == self.B_SELECT:
                string = "Button Select"
            if event.button == self.B_L3:
                string = "Button L3"
            if event.button == self.B_R3:
                string = "Button R3"
            if event.button == self.B_START:
                string = "Button Start"
            if event.button == self.B_UP:
                string = "Button UP"
            if event.button == self.B_RIGHT:
                string = "Button RIGHT"
            if event.button == self.B_DOWN:
                string = "Button DOWN"
            if event.button == self.B_LEFT:
                string = "Button LEFT"
            if event.button == self.B_L2:
                string = "Button L2"
            if event.button == self.B_R2:
                string = "Button R2"
            if event.button == self.B_L1:
                string = "Button L1"
            if event.button == self.B_R1:
                string = "Button R1"
            if event.button == self.B_TRI:
                string = "Button Triangle"
            if event.button == self.B_CIRC:
                string = "Button Circle"
            if event.button == self.B_X:
                string = "Button Xis"
            if event.button == self.B_SQR:
                string = "Button Square"
            if event.button == self.B_PS:
                string = "Button PS"

        if event.type == pygame.JOYAXISMOTION:
            stringType = "JOYAXISMOTION"
            return (stringType, string, event.value)
        if event.type == pygame.JOYBUTTONUP:
            stringType = "JOYBUTTONUP"
            return (stringType, string, 0)
        if event.type == pygame.JOYBUTTONDOWN:
            stringType = "JOYBUTTONDOWN"
            return (stringType, string, 1)  

