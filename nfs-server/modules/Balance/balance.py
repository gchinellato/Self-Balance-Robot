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

        self.speedKp = SPEED_KP
        self.speedKi = SPEED_KD
        self.speedKd = SPEED_KI
        self.angleKpCons = ANGLE_KP_CONS
        self.angleKiCons = ANGLE_KI_CONS
        self.angleKdCons = ANGLE_KD_CONS
        self.angleKpAggr = ANGLE_KP_AGGR
        self.angleKiAggr = ANGLE_KI_AGGR
        self.angleKdAggr = ANGLE_KD_AGGR

        self.speedPID = PID("Speed", SPEED_SETPOINT, self.speedKp, self.speedKi, self.speedKd, debug)  
        self.anglePID = PID("Angle", ANGLE_SETPOINT, self.angleKpCons, self.angleKiCons, self.angleKdCons, debug)
        self.anglePIDmode = PID_CONSERVATIVE 

        #apagar *************************
        self.offset = 0

        logging.info("Balance Module initialized")      

    #Override method
    def run(self):
        logging.info("Balance Thread started") 
        lastTime = 0.0
        runSpeed = 0.0
        turnSpeed = 0.0
        pitchPID = 0.0
        speedL = 0.0
        speedR = 0.0
        velocity = 0.0

        maxTime = 0
        duration = 0

        while not self._stopEvent.wait(self._sleepPeriod):
            self._lock.acquire() 

            currentTime = time.time()

            #Calculate time since last time was called
            #if (self.debug & MODULE_BALANCE):
            #    logging.debug("Duration: " + str(currentTime - lastTime))

            #
            # Complementary filter
            #

            #Calculate Pitch, Roll and Yaw
            self.imu.complementaryFilter(CF, self._sleepPeriod) 

            #
            # Motion
            #
            
            #Update wheel velocity each loop, then it is possible to getWheelVelocity
            self.motion.updateWheelVelocity(dt=self._sleepPeriod)   
            velocity = self.motion.getWheelVelocity()       

            #Get event for motion, ignore if empty queue                
            event = self.getEvent()
            if event != None:
                if event[0] != None:                        
                    runSpeed = self.motion.convertRange(event[0]) 
                if event[1] != None:
                    turnSpeed = self.motion.convertRange(event[1])

            self.speedPID.setSetpoint(runSpeed)
            output = self.speedPID.compute(velocity, self._sleepPeriod) 
            self.anglePID.setSetpoint(output + self.offset)

            #Select PID mode and check angles boundaries
            if ((self.anglePIDmode == PID_AGGRESSIVE) and (abs(self.imu.CFanglePitch) < ANGLE_LIMIT)):
                #almost in the setpoint, change to conservative mode
                self.anglePIDmode = PID_CONSERVATIVE
                self.anglePID.setTunings(self.angleKpCons, self.angleKiCons, self.angleKdCons)
            elif ((self.anglePIDmode == PID_CONSERVATIVE) and (abs(self.imu.CFanglePitch) >= ANGLE_LIMIT)):
                #so far from the setpoint, change to aggressive mode
                self.anglePIDmode = PID_AGGRESSIVE
                self.anglePID.setTunings(self.angleKpAggr, self.angleKiAggr, self.angleKdAggr)

            if (abs(self.imu.CFanglePitch) > ANGLE_IRRECOVERABLE):
                #so sorry, it is licking the floor :(
                speedL = 0.0
                speedR = 0.0
                pitchPID = 0.0
                self.motion.motorStop()
            else:
                pitchPID = self.anglePID.compute(round(self.imu.CFanglePitch,2),  self._sleepPeriod)
                speedL = pitchPID - turnSpeed
                speedR = pitchPID + turnSpeed
                self.motion.motorMove(speedL, speedR)

            #UDP message   
            #(module)(timestamp),(data1)(data2),(data3)(...)(#)
            UDP_MSG = "BALANCE" + "," + \
                      str(datetime.datetime.now()) + "," + \
                      str(round(self.imu.CFangleRoll,2)) + "," + \
                      str(round(self.imu.CFanglePitch,2)) + "," + \
                      str(round(self.imu.CFangleYaw,2)) + "," + \
                      str(round(runSpeed,2)) + "," + \
                      str(round(turnSpeed,2)) + "," + \
                      str(round(speedL,2)) + "," + \
                      str(round(speedR,2)) + "," + \
                      str(round(velocity,2)) + "#"
                   
            # Sending UDP packets...
            if (self.callbackUDP != None):
                self.callbackUDP(UDP_MSG) 
        
            lastTime = currentTime
            self._lock.release()
        
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        logging.info("Killing Balance Thread...") 
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

