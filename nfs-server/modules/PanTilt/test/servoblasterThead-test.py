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

############################################################

eventQueue = Queue.Queue()
#panTiltQueue = multiprocessing.Queue()
panTiltQueue = Queue.Queue()

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


