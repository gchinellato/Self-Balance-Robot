#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Manager all threads/processes
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

from Comm.UDP.UDP_Client import UDP_ClientThread
from Comm.Bluetooth.controller_ps3 import PS3_ControllerThread
from Balance import BalanceThread
from PanTilt.panTilt import PanTiltThread
from CV.cv import ComputerVisionThread
from IMU.constants import *
from Utils.traces.trace import *
import time
import RPi.GPIO as GPIO
import Queue
import multiprocessing
import pygame

CLIENT_UDP_NAME = "Client-UDP-Thread"
BALANCE_NAME = "Balance-Thread"
PAN_TILT_NAME = "PanTilt-Thread"
PS3_CTRL_NAME = "PS3-Controller-Thread"
TRACKING_NAME = "Tracking-Process"

def queueSize(q1, q2, q3, q4):
    logging.info("Queue Size - Client UDP: " + str(q1.qsize()))
    logging.info("Queue Size - Event: " + str(q2.qsize()))
    logging.info("Queue Size - Balance: " + str(q3.qsize()))
    logging.info("Queue Size - PanTilt: " + str(q4.qsize()))

def Manager():
    try:
        #Message queues to communicate between threads
        clientUDPQueue = Queue.Queue()
        eventQueue = Queue.Queue()
        #eventQueue = multiprocessing.Queue()
        balanceQueue = Queue.Queue(16)
        panTiltQueue = Queue.Queue()

        logging.info("Starting threads and process...")
        threads = []        

        #UDP Client thread
        clientUDP = UDP_ClientThread(name=CLIENT_UDP_NAME, queue=clientUDPQueue, debug=False, UDP_IP="192.168.1.35", UDP_PORT=5000)
        clientUDP.daemon = True
        threads.append(clientUDP)
        clientUDP.start()

        #Joystick thread
        joy = PS3_ControllerThread(name=PS3_CTRL_NAME, queue=eventQueue, debug=False)
        joy.daemon = True
        threads.append(joy)
        joy.start()

        #Balance thread
        balance = BalanceThread(name=BALANCE_NAME, queue=balanceQueue, debug=False, clientUDP=clientUDP)
        balance.daemon = True
        threads.append(balance)
        balance.start()

        #Computer Vision thread
        tracking = ComputerVisionThread(name=TRACKING_NAME, queue=eventQueue, debug=False)
        tracking.daemon = True
        #threads.append(tracking)
        #tracking.start()

        #Pan-Tilt thread
        panTilt = PanTiltThread(name=PAN_TILT_NAME, queue=panTiltQueue, debug=True)
        panTilt.daemon = True
        threads.append(panTilt)
        panTilt.start() 

        #Initialize Pan-Tilt
        headV = 0.0
        headH = 0.0
        #panTilt.putEvent((headV, headH))        

        runSpeed = 0.0
        turnSpeed = 0.0

        lastTime = 0.0
        LP = 0.0

        while True: 
            try:
                currentTime = time.time()

                #Calculate time since the last time it was called
                #logging.debug("Duration: " + str(currentTime - lastTime))
                
                #queueSize(clientUDPQueue, eventQueue, balanceQueue, panTiltQueue)
                event = eventQueue.get(timeout=2) 
                if event != None: 
                    if event[0] == PS3_CTRL_NAME and joy.joyStatus != None:
                        if (event[1].type == pygame.JOYAXISMOTION) and (event[1].axis != joy.A_ACC_X) and (event[1].axis != joy.A_ACC_Y) and (event[1].axis != joy.A_ACC_Z):
                            if event[1].axis == joy.A_R3_V:
                                headV = event[1].value
                                #panTilt.putEvent((headV, None))
                                #logging.debug(("R3 Vertical: {0}, {1}, {2}".format(event[1].axis, event[1].value, headV)))
                            if event[1].axis == joy.A_R3_H:
                                headH = -event[1].value 
                                #panTilt.putEvent((None, headH))
                                #logging.debug(("R3 Horizontal: {0}, {1}, {2}".format(event[1].axis, event[1].value, headH)))

                            if event[1].axis == joy.A_L3_V:
                                runSpeed = event[1].value
                                balance.putEvent((runSpeed, None)) 
                                #logging.debug(("L3 Vertical: {0}, {1}, {2}".format(event[1].axis, event[1].value, runSpeed)))
                            if event[1].axis == joy.A_L3_H: 
                                turnSpeed = event[1].value
                                balance.putEvent((None, turnSpeed)) 
                                #logging.debug(("L3 Horizontal: {0}, {1}, {2}".format(event[1].axis, event[1].value, turnSpeed)))
                        
                        if event[1].type == pygame.JOYBUTTONDOWN or event[1].type == pygame.JOYBUTTONUP:
                            #logging.debug(joy.parseEvent(event[1]))
                            if event[1].button == joy.B_SQR:
                                logging.debug("Button Square")
                    #IP controller
                    #elif event[0] == SERVER_UDP_NAME:                
                    #OpenCV controller            
                    elif event[0] == TRACKING_NAME:                        
                        tracking.block.set() 

                        #Delta measure from object up to center of the vision
                        dWidth, dHeight = event[1]   
                        logging.debug(("Distance center X: {0}, Y: {1}".format(dWidth, dHeight)))  

                        #Get relative angles
                        angleV, angleH = panTilt.getRelativeAngles()  

                        #Vertical
                        if dHeight < -100 or dHeight > 100:
                            if dHeight < -100:
                                angle = -10.0
                                if dHeight < -200:
                                    angle = -20.0  
                            elif dHeight > 100:
                                angle = 10.0
                                if dHeight > 200:
                                    angle = 20.0
                            headV = panTilt.convertDegreeToAnalogValue(angleV+angle)
                            #panTilt.putEvent((headV, None))

                        #Horizontal
                        if dWidth < -100 or dWidth > 100:
                            if dWidth < -100:
                                angle = 10.0
                                if dWidth < -200:
                                    angle = 20.0                             
                            elif dWidth > 100:
                                angle = -10.0
                                if dWidth > 200:
                                    angle = -20.0
                            headH = panTilt.convertDegreeToAnalogValue(angleH+angle)
                            #panTilt.putEvent((None, headH))

                        logging.debug(("Angles relative: " + str(panTilt.getRelativeAngles())))
                        #logging.debug(("Angles absolute: " + str(panTilt.getAbsoluteAngles())))
                        tracking.block.clear()
              
            except Queue.Empty:
                #logging.debug("Queue Empty")
                pass
            finally:
                lastTime = currentTime
                time.sleep(LP)
    finally:
        logging.info("########################################")
        logging.info("Exiting...")
        for t in threads:
            logging.info("Killing "+ str(t.name) + "...")
            t.join()
        GPIO.cleanup()
        logging.info("GPIO cleanup...")

if __name__ == '__main__':
    Manager() 
