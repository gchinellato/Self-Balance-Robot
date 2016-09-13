#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: UDP Server Thread
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import sys
import time
import socket
import threading
is_py2 = sys.version[0] == '2'
if is_py2:
    import Queue as queue
else:
    import queue as queue
import logging
from Utils.traces.trace import *

class UDP_ServerThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=queue.Queue(), debug=0, UDP_IP="", UDP_PORT=5000):
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
        self._sleepPeriod = 0.0
        
        #UDP server config
        self.UDP_IP = UDP_IP 
        self.UDP_PORT = UDP_PORT  

        logging.info("UDP Server Module initialized") 
    
    #Override method
    def run(self):
        logging.info("UDP Server Thread Started")
        #Open socket through UDP/IP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT)) 
        self.sock.settimeout(2.0)
        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                self._lock.acquire()

                currentTime = time.time()
                #Calculate time since the last time it was called
                #if (self.debug & MODULE_SERVER_UDP):
                #    logging.debug("Duration: " + str(currentTime - lastTime))

                strData, addr = self.sock.recvfrom(128)  
                strData = strData.decode("utf-8")   
                data = self.parseData(strData) 
                self.putMessage(self.name, data)
            except queue.Full:
                if (self.debug & MODULE_SERVER_UDP):
                    logging.debug("Queue Full")
                pass 
            except socket.timeout:
                if (self.debug & MODULE_SERVER_UDP):
                    logging.debug("Socket Timeout")
                pass 
            finally:
                lastTime = currentTime
                self._lock.release()
    
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        logging.info("Killing UDP Server Thread...") 
        self._stopEvent.set()
        self.sock.close()
        threading.Thread.join(self, timeout=timeout)

    def getMessage(self, timeout=2):
        return self._workQueue.get(timeout=timeout)

    def putMessage(self, name, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():       
            self._workQueue.put((name, msg))

    def parseData(self, strData):
        #Check if message is completed    
        if strData.find("#") != -1:
            #Remove end char
            strData = strData.replace("#","")
            data = strData.split(",")
            return data
        else:
            logging.warning("Uncompleted UDP message.")
            return None

LP = 0.0

def main():
    try:
        serverThread = UDP_ServerThread(name="Thread-UDP-Server", debug=False)
        serverThread.daemon = True
        serverThread.start()

        while True:
            UDP_MSG = serverThread.getMessage()
            if UDP_MSG != None: 
                logging.info(UDP_MSG) 
            time.sleep(LP)

    except KeyboardInterrupt: 
        logging.info("Exiting...") 
        serverThread.join()  

if __name__ == "__main__":
    main()
