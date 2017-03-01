#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Description: Serial API
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

import threading
import datetime
import time
import serial
import Queue as queue
from constants import *
from Utils.traces.trace import *
from Utils.constants import *

class SerialThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=queue.Queue(), debug=0, COM="/dev/ttyUSB0", callbackUDP=None):
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
        self._sleepPeriod = 0.00

        self.COM = COM
        self.port = None

        logging.info("Serial Module initialized")

    #Override method
    def run(self):
        logging.info("Serial Thread Started")

        self.port = serial.Serial(self.COM, timeout=1, baudrate=38400)
        logging.info(self.port.name)

        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            currentTime = time.time()

            #Calculate time since the last time it was called
            #if (self.debug & MODULE_SERIAL):
            #    logging.debug("Duration: " + str(currentTime - lastTime))

            msg = self.getMessage()
            if msg != None:
                size = self.port.write(''.join(msg))
                if (self.debug & MODULE_SERIAL):
                    logging.debug(("Writing to Arduino >>>: " + str(msg) + ", " + str(len(msg))))

            #Read trace from arduino
            recv = self.port.readline()

            if (self.debug & MODULE_SERIAL):
                logging.debug(("Reading from Arduino <<<: " + str(recv)))

            #Parse event
            msgList = self.parseData(recv)
            UDP_MSG = None

            if msgList != None:
                #UDP message
                #(module),(data1),(data2),(data3),(...)(#)
                UDP_MSG = CMD_SERIAL
                for msg in msgList:
                    UDP_MSG += ("," + msg)
                UDP_MSG += "#"

            #Sending UDP packets...
            if (self.callbackUDP != None and UDP_MSG != None):
                self.callbackUDP(UDP_MSG)

            lastTime = currentTime

    #Override method
    def join(self, timeout=2):
        #Stop the thread and wait for it to end
        logging.info("Killing Serial Thread...")
        self._stopEvent.set()
        self.port.close()
        threading.Thread.join(self, timeout=timeout)

    def getMessage(self, timeout=2):
        #Bypass if empty, to not block the current thread
        if not self._workQueue.empty():
            return self._workQueue.get(timeout=timeout)
        else:
            return None

    def putMessage(self, command, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():
            msg = self.checkData(command, msg)
            self._workQueue.put(msg)

    def parseData(self, strData):
        #Remove begin and end chars
        strData = strData.replace("\r","")
        strData = strData.replace("\n","")
        data = strData.split(",")
        return data

    def checkData(self, command, msg):
        '''(SIZE),(COMMAND),(PARAM_1),(PARAM_2),(...),(END_CHAR)'''

        ret = str(command) + ","

        if command == STARTED:
            ret += str(msg)
        elif command == DIRECTION:
            param = self.convertTo(msg, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
            ret += str(round(param,2))
        elif command == STEERING:
            param = self.convertTo(msg, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
            ret += str(round(param,2))
        elif command == SPEED_PID:
            ret += str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID_AGGR:
            ret += str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID_CONS:
            ret += str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == CALIBRATED_ZERO_ANGLE:
            ret += str(round(msg,2))
        elif command == ANGLE_LIMIT:
            ret += str(round(msg,2))
        else:
            ret += "unknown"

        ret += "\n"

        msgSize = len(ret)
        digits = len(str(msgSize))
        size = msgSize + digits

        if len(str(msgSize)) != len(str(size)):
             size += (size - msgSize)

        return str(size) + "," + ret

    def converStrToHex(self, msg):
        return ":".join("{:02x}".format(ord(c)) for c in msg)

    def convertTo(self, value, fromMax, fromMin, toMax, toMin):
        if not value >= fromMin and value <= fromMax:
            logging.warning("Value out of the range (Max:"+str(fromMax)+" , Min:"+str(fromMin)+")")
            if value > fromMax:
                value = fromMax
            elif value < fromMin:
                value = fromMin

        factor = (value-fromMin)/(fromMax-fromMin)
        return factor*(toMax-toMin)+toMin
