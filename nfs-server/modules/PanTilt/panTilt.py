#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Description: Pan Tilt - Micro Servo motors API with Servoblaster
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

import os
import time
import threading
import Queue
import datetime
from constants import *
from Utils.gpio_mapping import *
from Utils.traces.trace import *
from Utils.constants import *

class PanTiltThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=0, callbackUDP=None):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug
        self.callbackUDP = callbackUDP

        #Queue to communicate between threads
        self._workQueue = queue

        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.0

        #Angles
        self.angleV = 0.0
        self.angleH = 0.0
        self.scaledAngleV = 0.0
        self.scaledAngleH = 0.0

        #See gpio_mapping.py to check pins 
        os.system('sudo servod --p1pins=7,11')
        time.sleep(0.1)

        logging.info("Pan-Tilt Module initialized")

    #Override method
    def run(self):
        logging.info("Pan-Tilt Thread started")

        #Initial position
        pwmVertical = self.convertTo(0, ANALOG_MAX, ANALOG_MIN, VERTICAL_MAX, VERTICAL_MIN)
        self.angleV = self.convertTo(pwmVertical, POS_MAX, POS_MIN, ANGLE_MAX, ANGLE_MIN)
        self.scaledAngleV = self.convertTo(pwmVertical, VERTICAL_MAX, VERTICAL_MIN, ANGLE_MAX, ANGLE_MIN)
        self._changeV(pwmVertical)

        pwmHorizontal = self.convertTo(0, ANALOG_MAX, ANALOG_MIN, HORIZONTAL_MAX, HORIZONTAL_MIN)
        self.angleH = self.convertTo(pwmHorizontal, POS_MAX, POS_MIN, ANGLE_MAX, ANGLE_MIN)
        self.scaledAngleH = self.convertTo(pwmHorizontal, HORIZONTAL_MAX, HORIZONTAL_MIN, ANGLE_MAX, ANGLE_MIN)
        self._changeH(pwmHorizontal)

        time.sleep(0.1)

        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                currentTime = time.time()

                #Calculate time since the last time it was called
                #if (self.debug):
                #    logging.debug("Duration: " + str(currentTime - lastTime))

                event = self.getEvent()
                if event != None:
                    if event[0] != None:
                        pwmVertical = self.convertTo(event[0], ANALOG_MAX, ANALOG_MIN, VERTICAL_MAX, VERTICAL_MIN)
                        self.angleV = self.convertTo(pwmVertical, POS_MAX, POS_MIN, ANGLE_MAX, ANGLE_MIN)
                        self.scaledAngleV = self.convertTo(pwmVertical, VERTICAL_MAX, VERTICAL_MIN, ANGLE_MAX, ANGLE_MIN)
                        self._changeV(pwmVertical)

                        if (self.debug & MODULE_PANTILT):
                            logging.debug("PWM Vertical: " + str(pwmVertical) + "%")
                            logging.debug("Angle Vertical: " + str(self.angleV) + "deg")
                            logging.debug("Angle Scaled Vertical: " + str(self.scaledAngleV) + "deg")

                    if event[1] != None:
                        pwmHorizontal = self.convertTo(event[1], ANALOG_MAX, ANALOG_MIN, HORIZONTAL_MAX, HORIZONTAL_MIN)
                        self.angleH = self.convertTo(pwmHorizontal, POS_MAX, POS_MIN, ANGLE_MAX, ANGLE_MIN)
                        self.scaledAngleH = self.convertTo(pwmHorizontal, HORIZONTAL_MAX, HORIZONTAL_MIN, ANGLE_MAX, ANGLE_MIN)
                        self._changeH(pwmHorizontal)

                        if (self.debug & MODULE_PANTILT):
                            logging.debug("PWM Horizontal: " + str(pwmHorizontal) + "%")
                            logging.debug("Angle Horizontal: " + str(self.angleH) + "deg")
                            logging.debug("Angle Scaled Horizontal: " + str(self.scaledAngleH) + "deg")

                    #UDP message
                    #(module)(data1),(data2),(data3),(...)(#)
                    UDP_MSG = CMD_PAN_TILT + "," + \
                              str(round(pwmVertical,2)) + "," + \
                              str(round(pwmHorizontal,2)) + "#"

                    #Sending UDP packets...
                    if (self.callbackUDP != None):
                        self.callbackUDP(UDP_MSG)

            except Queue.Empty:
                if (self.debug & MODULE_PANTILT):
                    logging.debug("Queue Empty")
                pass
            finally:
                lastTime = currentTime

    #Override method
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        logging.info("Killing PanTilt Thread...")
        self._stopEvent.set()
        os.system('sudo killall servod')
        threading.Thread.join(self, timeout=timeout)

    def getEvent(self, timeout=1):
        return self._workQueue.get(timeout=timeout)

    def putEvent(self, event):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():
            self._workQueue.put(event)

    def getAbsoluteAngles(self):
        #Get absolute angle - real angle
        return self.angleV, self.angleH

    def getScaledAngles(self):
        #Get angle relative the limits
        return self.scaledAngleV, self.scaledAngleH

    def convertTo(self, value, fromMax, fromMin, toMax, toMin):
        if not (value > fromMin and value < fromMax):
            logging.warning("Value out of the range (Max:"+str(fromMax)+" , Min:"+str(fromMin)+")")

        if value > fromMax:
            value = fromMax
        elif value < fromMin:
            value = fromMin

        factor = (value-fromMin)/(fromMax-fromMin)
        return factor*(toMax-toMin)+toMin

    def _changeV(self, percentage):
        servoBlaster = open('/dev/servoblaster', 'w')
        servoBlaster.write(SERVO_V + '=' + str(percentage) + '%' + '\n')
        servoBlaster.flush()
        servoBlaster.close()

    def _changeH(self, percentage):
        servoBlaster = open('/dev/servoblaster', 'w')
        servoBlaster.write(SERVO_H + '=' + str(percentage) + '%' + '\n')
        servoBlaster.flush()
        servoBlaster.close()
