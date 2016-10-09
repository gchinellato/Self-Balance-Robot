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
        self._lock = threading.Lock()

        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.0

        self.COM = COM
        self.port = None

        logging.info("Serial Module initialized")

    #Override method
    def run(self):
        logging.info("Serial Thread Started")

        try:
            self.port = serial.Serial(self.COM, baudrate=19200)
            logging.info(self.port.name)

            lastTime = 0.0
            i = 0

            while not self._stopEvent.wait(self._sleepPeriod):
                try:
                    self._lock.acquire()

                    currentTime = time.time()

                    #Calculate time since the last time it was called
                    #if (self.debug & MODULE_SERIAL):
                    #    logging.debug("Duration: " + str(currentTime - lastTime))

                    msg = self.getMessage()
                    if msg != None:
                        size = self.port.write(''.join(msg))

                        if (self.debug & MODULE_SERIAL):
                            logging.debug(("Writing to Arduino: " + str(msg)))
                            #logging.debug(("Writing to Arduino: " + self.converStrToHex(msg)))

                        '''#read ack from arduino and check the size of the received message
                        recvAck = self.port.readline()

                        if (self.debug & MODULE_SERIAL):
                            logging.debug(("Reading from Arduino ACK: " + str(recvAck)))
                            #logging.debug(("Reading to Arduino ACK: " + self.converStrToHex(str(recvAck))))

                        if (self.debug & MODULE_SERIAL):
                            if(len(msg) == len(recvAck)):
                                logging.debug("ACK OK: Written size: " + str(size) + ", read size: " + str(len(recvAck)))
                            else:
                                logging.debug("ACK NOK: Written size: " + str(size) + ", read size: " + str(len(recvAck)))

                        #read command from arduino
                        recvCmd = self.port.readline()

                        if (self.debug & MODULE_SERIAL):
                            logging.debug(("Reading from Arduino command: " + str(recvCmd)))
                            #logging.debug(("Reading to Arduino ACK: " + self.converStrToHex(str(recvCmd))))*/'''

                    #Read trace from arduino
                    recv = self.port.readline()

                    if (self.debug & MODULE_SERIAL):
                        logging.debug(("Reading from Arduino: " + str(recv)))
                        #logging.debug(("Reading to Arduino: " + self.converStrToHex(str(recv))))

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

                except queue.Empty:
                    if (self.debug & MODULE_SERIAL):
                        logging.debug("Queue Empty")
                    pass
                finally:
                    lastTime = currentTime
                    self._lock.release()
        except serial.SerialException:
            if (self.debug & MODULE_SERIAL):
                logging.debug("Serial Exception")

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
        #Check if message is completed
        if (TRACE_BEGIN in strData) and (TRACE_END in strData):
            #Remove begin and end chars
            strData = strData.replace(TRACE_BEGIN,"")
            strData = strData.replace(TRACE_END,"")
            strData = strData.replace("\r","")
            strData = strData.replace("\n","")
            data = strData.split(",")
            return data
        else:
            #logging.warning("Invalid message")
            return None

    def checkData(self, command, msg):
        '''(TRACE_BEGIN)(COMMAND),(NUM_PARAM),(PARAM_1),(PARAM_2),(...)(TRACE_END)'''
        if command == STARTED:
            msg = str(command) + "," + "1" + "," + str(msg)
        elif command == DIRECTION:
            msg = self.convertTo(msg, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        elif command == STEERING:
            msg = self.convertTo(msg, ANALOG_MAX, ANALOG_MIN, PWM_MAX, PWM_MIN)
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        elif command == SPEED_PID:
            msg = str(command) + "," + "3" + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID_AGGR:
            msg = str(command) + "," + "3" + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID_CONS:
            msg = str(command) + "," + "3" + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == CALIBRATED_ZERO_ANGLE:
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        elif command == ANGLE_LIMIT:
            msg = str(command) + "," + "1" + "," + str(round(msg,2))
        else:
            msg = "unknown"

        return TRACE_BEGIN + msg + TRACE_END + "\r\n"

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

def main():
    try:
        setVerbosity("debug")

        queueToWrite = queue.Queue()

        serialThread = SerialThread(name="Thread-Serial", queue=queueToWrite, debug=MODULE_SERIAL)
        serialThread.start()

        i = 0

        while True:
            logging.info("Seding packet: " + str(i))
            serialThread.putMessage("TESTE", "ABC")
            i += 1
            time.sleep(1)

    except KeyboardInterrupt:
        logging.info("Exiting...")
        serialThread.join()

if __name__ == '__main__':
    main()
