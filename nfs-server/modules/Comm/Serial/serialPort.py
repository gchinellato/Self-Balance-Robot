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
        self._sleepPeriod = 0.01    

        self.COM = COM 
        self.port = None 
 
        logging.info("Serial Module initialized") 
    
    #Override method
    def run(self):
        logging.info("Serial Thread Started")

        try:
            self.port = serial.Serial(self.COM, baudrate=115200, timeout=2.0)
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
                        self.port.write(''.join(msg))

                        if (self.debug & MODULE_SERIAL):
                            logging.debug(("Writing to Arduino: " + str(msg))) 

                    recv = self.port.readline()
        
                    logging.info(("Reading from Arduino: " + str(recv)))

                    #Parse event
                    msgList = self.parseData(recv)
                    UDP_MSG = None

                    if msgList != None:
                        #UDP message   
                        #(module)(timestamp),(data1)(data2),(data3)(...)(#)
                        UDP_MSG = "SERIAL" + "," + str(datetime.datetime.now())

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
    def join(self, timeout=None):
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

    def putMessage(self, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full(): 
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
            logging.warning("Invalid message")
            return None  

def main():
    try:
        setVerbosity("debug")

        queueToWrite = queue.Queue()

        serialThread = SerialThread(name="Thread-Serial", queue=queueToWrite, debug=MODULE_SERIAL)
        serialThread.start()    

        i = 0

        while True:
            logging.info("Seding packet: " + str(i)) 
            serialThread.putMessage("ABC")
            i += 1
            time.sleep(1)

    except KeyboardInterrupt: 
        logging.info("Exiting...") 
        serialThread.join() 

if __name__ == '__main__':
    main()
