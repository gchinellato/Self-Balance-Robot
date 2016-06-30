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

from Comm.UDP.UDP_Server import UDP_ServerThread
from Comm.UDP.UDP_Client import UDP_ClientThread
from Comm.Bluetooth.controller_ps3 import PS3_ControllerThread
from Balance.balance import BalanceThread
from PanTilt.panTilt import PanTiltThread
from ComputerVison.tracking import ComputerVisionThread
from IMU.constants import *
from PanTilt.constants import *
from Utils.traces.trace import *
import time
import RPi.GPIO as GPIO
import Queue
import multiprocessing
import pygame

CLIENT_UDP_NAME = "Client-UDP-Thread"
SERVER_UDP_NAME = "Server-UDP-Thread"
BALANCE_NAME = "Balance-Thread"
PAN_TILT_NAME = "PanTilt-Thread"
PS3_CTRL_NAME = "PS3-Controller-Thread"
TRACKING_NAME = "Tracking-Process"

def Manager():
    try:
        #Message queues to communicate between threads
        clientUDPQueue = Queue.Queue()
        eventQueue = Queue.Queue()
        balanceQueue = Queue.Queue()
        panTiltQueue = Queue.Queue()

        logging.info("Starting threads and process...")
        threads = []        

        #UDP Client thread
        clientUDP = UDP_ClientThread(name=CLIENT_UDP_NAME, queue=clientUDPQueue, debug=False, UDP_IP="192.168.1.35", UDP_PORT=5000)
        clientUDP.daemon = True
        threads.append(clientUDP)
        clientUDP.start()

        #UDP Server thread
        serverUDP = UDP_ServerThread(name=SERVER_UDP_NAME, queue=eventQueue, debug=False, UDP_IP="", UDP_PORT=5001)
        serverUDP.daemon = True
        threads.append(serverUDP)
        serverUDP.start()

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
        #tracking = ComputerVisionThread(name=TRACKING_NAME, queue=eventQueue, debug=True)
        #tracking.daemon = True
        #threads.append(tracking)
        #tracking.start()

        #Pan-Tilt thread
        panTilt = PanTiltThread(name=PAN_TILT_NAME, queue=panTiltQueue, debug=False)
        panTilt.daemon = True
        threads.append(panTilt)
        panTilt.start()      

        runSpeed = 0.0
        turnSpeed = 0.0

        lastTime = 0.0
        LP = 0.0

        while True: 
            try:
                currentTime = time.time()

                #Calculate time since the last time it was called
                #logging.debug("Duration: " + str(currentTime - lastTime))

                event = eventQueue.get(timeout=2) 
                if event != None: 
                    if event[0] == PS3_CTRL_NAME and joy.joyStatus != None:
                        if (event[1].type == pygame.JOYAXISMOTION) and (event[1].axis != joy.A_ACC_X) and (event[1].axis != joy.A_ACC_Y) and (event[1].axis != joy.A_ACC_Z):
                            #Head Vertical
                            if event[1].axis == joy.A_R3_V:
                                headV = event[1].value
                                panTilt.putEvent((headV, None))
                            #Head Horizontal
                            if event[1].axis == joy.A_R3_H:
                                headH = -event[1].value 
                                panTilt.putEvent((None, headH))
                            #Body run speed
                            if event[1].axis == joy.A_L3_V:
                                runSpeed = event[1].value
                                balance.putEvent((runSpeed, None)) 
                            #Body turn speed
                            if event[1].axis == joy.A_L3_H: 
                                turnSpeed = event[1].value
                                balance.putEvent((None, turnSpeed)) 
                        
                        if event[1].type == pygame.JOYBUTTONDOWN or event[1].type == pygame.JOYBUTTONUP:
                            if event[1].button == joy.B_SQR:
                                logging.debug("Button Square")
                    #IP controller
                    elif event[0] == SERVER_UDP_NAME:
                        logging.debug(event[1])
                        if event[1][1] == "PID": 
                            balance.Kp = float(event[1][2])
                            balance.Ki = float(event[1][3])
                            balance.Kd = float(event[1][4])
                            logging.debug(("PID: Kp: " + str(balance.Kp)))
                            logging.debug(("PID: Ki: " + str(balance.Ki)))
                            logging.debug(("PID: Kd: " + str(balance.Kd)))
                
                    #OpenCV controller            
                    elif event[0] == TRACKING_NAME:                        
                        tracking.block.set() 

                        #Delta measure from object up to center of the vision
                        dWidth, dHeight, radius = event[1]   
                        logging.debug(("Distance to center X: {0}, Y: {1}, Radius:{2}".format(dWidth, dHeight, radius)))  

                        #Get current head angles
                        angleV, angleH = panTilt.getScaledAngles() 
                        logging.debug("Angles current: " + str((angleV, angleH)))

                        #Head Vertical
                        if dHeight < -100 or dHeight > 100:
                            if dHeight < -100:
                                angle = -10.0
                                if dHeight < -200:
                                    angle = -20.0  
                            elif dHeight > 100:
                                angle = 10.0
                                if dHeight > 200:
                                    angle = 20.0
                            headV = panTilt.convertTo(angleV+angle, ANGLE_MAX, ANGLE_MIN, ANALOG_MAX, ANALOG_MIN)
                            panTilt.putEvent((headV, None))

                        #Head Horizontal
                        if dWidth < -100 or dWidth > 100:
                            if dWidth < -100:
                                angle = 10.0
                                if dWidth < -200:
                                    angle = 20.0                             
                            elif dWidth > 100:
                                angle = -10.0
                                if dWidth > 200:
                                    angle = -20.0
                            headH = panTilt.convertTo(angleH+angle, ANGLE_MAX, ANGLE_MIN, ANALOG_MAX, ANALOG_MIN)
                            panTilt.putEvent((None, headH))

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
