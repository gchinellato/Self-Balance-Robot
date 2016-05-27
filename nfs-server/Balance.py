#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: Balance Controller
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

from IMU.GY80_IMU import GY80_IMU
from Motion.motion import Motion
from IMU.constants import *
from Utils.traces.trace import *
import datetime
import time
import threading
import Queue

class BalanceThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(16), debug=False, clientUDP=None):
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

        #Create objects
        self.clientUDP = clientUDP
        self.angles = GY80_IMU(debug=False)
        self.motion = Motion(debug=False)

        logging.info("Balance Thread initialized")      

    #Override method
    def run(self):
        lastTime = 0.0
        runSpeed = 0.0
        turnSpeed = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            self._lock.acquire()           

            currentTime = time.time()

            #Calculate time since last time was called
            #if (self.debug):
            #    logging.info("Duration: " + str(currentTime - lastTime))
            
            #
            # Complementary filter
            #
            self.angles.complementaryFilter(CF, self._sleepPeriod) 

            #
            # Motion
            #
            if (self.angles.CFanglePitch < 60.0) and (self.angles.CFanglePitch > -50.0):
                pitchPID = self.motion.PID(setPoint=8.0, newValue=self.angles.CFanglePitch, Kp=5.0, Ki=0.1, Kd=0.1)

                #Get event for motion, ignore if empty queue
                event = self.getEvent()
                if event != None:
                    if event[0] != None:                        
                        runSpeed = self.motion.convertRange(event[0]) 
                    if event[1] != None:
                        turnSpeed = self.motion.convertRange(event[1])

                speedL = pitchPID + runSpeed - turnSpeed
                speedR = pitchPID + runSpeed + turnSpeed 
                self.motion.motorMove(speedL, speedR)
            else:
                self.motion.motorStop()

            #UDP message   
            #(timestamp),(data1)(data2),(data3)(#)
            UDP_MSG = str(datetime.datetime.now()) + "," + \
                      str(self.angles.CFangleRoll) + "," + \
                      str(self.angles.CFanglePitch) + "," + \
                      str(self.angles.CFangleYaw) + "," + \
                      str(pitchPID) + "#"
       
            # Sending UDP packets...
            self.clientUDP.putMessage(UDP_MSG) 
        
            lastTime = currentTime
            self._lock.release()
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        self._stopEvent.set()
        self.motion.motorShutDown() 
        threading.Thread.join(self, timeout=timeout) 

    def getEvent(self):
        #Bypass if empty, to not block the current thread
        if not self._workQueue.empty():
            return self._workQueue.get()
        else:
            return None  

    def putEvent(self, event):   
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():       
            self._workQueue.put(event)    

