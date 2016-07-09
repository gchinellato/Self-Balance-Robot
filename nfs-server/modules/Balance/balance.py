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
from Motion.PID.PID import PID
from Motion.constants import *
from IMU.constants import *
from Utils.traces.trace import *
import datetime
import time
import threading
import Queue

class BalanceThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(16), debug=0, callbackUDP=None):
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
        self.callbackUDP = callbackUDP
        self.imu = GY80_IMU(debug=debug)
        self.motion = Motion(debug=debug)
        self.speedPID = PID("Speed", SPEED_SETPOINT, SPEED_KP, SPEED_KI, SPEED_KD, debug)  
        self.anglePID = PID("Angle", ANGLE_SETPOINT, ANGLE_KP_CONS, ANGLE_KI_CONS, ANGLE_KD_CONS, debug) 

        logging.info("Balance Thread initialized")      

    #Override method
    def run(self):
        lastTime = 0.0
        runSpeed = 0.0
        turnSpeed = 0.0
        pitchPID = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            self._lock.acquire()           

            currentTime = time.time()

            #Calculate time since last time was called
            #if (debug & MODULE_BALANCE):
            #    logging.info("Duration: " + str(currentTime - lastTime))

            #
            # Complementary filter
            #

            #Calculate Pitch, Roll and Yaw
            self.imu.complementaryFilter(CF, self._sleepPeriod) 

            #
            # Motion
            #
            
            #Update wheel velocity each loop, then it is possible read wheelVelocity
            self.motion.updateWheelVelocity()              

            #Get event for motion, ignore if empty queue                
            event = self.getEvent()
            if event != None:
                if event[0] != None:                        
                    runSpeed = self.motion.convertRange(event[0]) 
                if event[1] != None:
                    turnSpeed = self.motion.convertRange(event[1])

            self.speedPID.setpoint = runSpeed

            pitchPID = self.anglePID.compute(self.imu.CFanglePitch,  self._sleepPeriod)
            #self.anglePID.setpoint = self.speedPID.compute(self.motion.wheelVelocity,  self._sleepPeriod)

            #Select PID mode and check angles boundaries
            '''if ((self.anglePID.modePID == PID_AGGRESSIVE) and (abs(self.imu.CFanglePitch) < self.anglePID.anglePIDLimit)):
                #almost in the setpoint, change to conservative mode
                self.anglePID.modePID = PID_CONSERVATIVE
                self.anglePID.setTunings(ANGLE_KP_CONS, ANGLE_KI_CONS, ANGLE_KD_CONS)
            elif ((self.anglePID.modePID == PID_CONSERVATIVE) and (abs(self.imu.CFanglePitch) >= self.anglePID.anglePIDLimit)):
                #so far from the setpoint, change to aggressive mode
                self.anglePID.modePID = PID_AGGRESSIVE
                self.anglePID.setTunings(ANGLE_KP_AGGR, ANGLE_KI_AGGR, ANGLE_KD_AGGR)
            elif (abs(self.imu.CFanglePitch) > ANGLE_IRRECOVERABLE):
                #so sorry, it is licking the floor :(
                self.motion.motorStop()
            else:
                logging.debug("Keep the parameters")'''

            #pitchPID = self.anglePID.compute(self.imu.CFanglePitch,  self._sleepPeriod)

            speedL = pitchPID +runSpeed- turnSpeed/8
            speedR = pitchPID +runSpeed+ turnSpeed/8 
            self.motion.motorMove(speedL, speedR)

            #UDP message   
            #(timestamp),(data1)(data2),(data3)(#)
            UDP_MSG = str(datetime.datetime.now()) + "," + \
                      str(self.imu.CFangleRoll) + "," + \
                      str(self.imu.CFanglePitch) + "," + \
                      str(self.imu.CFangleYaw) + "," + \
                      str(round(pitchPID,2)) + "," + \
                      str(round(runSpeed,2)) + "," + \
                      str(round(turnSpeed,2)) + "," + \
                      str(round(speedL,2)) + "," + \
                      str(round(speedR,2)) + "#"
                   
            # Sending UDP packets...
            if (self.callbackUDP != None):
                self.callbackUDP(UDP_MSG) 
        
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

