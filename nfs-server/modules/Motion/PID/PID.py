#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: PID module
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import time
from Motion.constants import *
from Utils.traces.trace import *

class PID():
    def __init__(self, name, setpoint, Kp, Ki, Kd, debug=0):
        self.debug = debug
        self.name = name

        self.setpoint = setpoint
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._Cp = 0
        self._Ci = 0
        self._Cd = 0
        self.modePID = PID_CONSERVATIVE
        self.anglePIDLimit = ANGLE_LIMIT

        self.lastTime = 0
        self.lastError = 0

        logging.info(("PID " + str(name) + " module initialized"))  

    def compute(self, inputValue, dt):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        currentTime = time.time()

        #Calculate delta time
        dt = currentTime - self.lastTime

        #Calculate error
        error = self.setpoint - inputValue

        #Calculate delta error
        de = error - self.lastError

        #Proportional Term          
        self._Cp = error
      
        #Integral Term
        self._Ci += error*dt
 
        #Windup guard for Integral term do not reach very large values
        if self._Ci > WINDUP_GUARD:
            self._Ci = WINDUP_GUARD
        elif self._Ci < -WINDUP_GUARD:
            self._Ci = -WINDUP_GUARD
       
        self._Cd = 0
        #to avoid division by zero
        if dt > 0:
            #Derivative term
            self._Cd = de/dt 

        #Save for the next iteration
        self.lastError = error
        self.lastTime = currentTime        

        #Sum terms
        output = (self._Cp * self.Kp) + (self._Ci * self.Ki) + (self._Cd * self.Kd)

        if (self.debug & MODULE_MOTION_PID): 
            logging.debug(("PID output = %0.2f, input: %0.2f, error: %0.2f, Cp: %0.2f, Ci: %0.2f, Cd: %0.2f" % (output, inputValue, error, self._Cp, self._Ci, self._Cd)))

        return output

    def setSetpoint(self, setpoint):
        """ Set setpoint """
        self.setpoint = setpoint

        if (self.debug & MODULE_MOTION_PID): 
            logging.debug(("New PID %s parameters: Setpoint: %0.2f" % (self.name, self.setpoint)))

    def setTunings(self, Kp, Ki, Kd):
        """ Set PID parameters """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        if (self.debug & MODULE_MOTION_PID): 
            logging.debug(("New PID %s parameters: Kp: %0.2f, Ki: %0.2f, Kd: %0.2f" % (self.name, self.Kp, self.Ki, self.Kd)))

    def getParameters(self):
        return (self.setpoint, self.Kp, self.Ki, self.Kd)

def TestPID():
    try:
        setVerbosity("debug") 
        anglePID = PID("Angle", ANGLE_SETPOINT, ANGLE_KP, ANGLE_KI, ANGLE_KD, MODULE_MOTION_PID)
        anglePID.setTunings(ANGLE_SETPOINT, ANGLE_KP, ANGLE_KI, ANGLE_KD)

        print "PID parameters: " + str(anglePID.getParameters())

        LP = 0.01

        for i in range(100):
            anglePID.compute(i, LP)
            time.sleep(LP)

    except KeyboardInterrupt:
        print "Exiting..."

if __name__ == '__main__':
    TestPID()

