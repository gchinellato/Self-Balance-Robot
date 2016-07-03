#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance                         
* @Description: UDP Client Thread
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import time
import datetime
import socket
import threading
import Queue
from Utils.traces.trace import *

class UDP_ClientThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=0, UDP_IP="192.168.1.35", UDP_PORT=5000):
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
        
        #UDP Client config
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT

        logging.info("UDP Client Thread initialized") 
    
    #Override method
    def run(self):
        #Open socket through UDP/IP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        lastTime = 0.0

        while not self._stopEvent.wait(self._sleepPeriod):
            try: 
                self._lock.acquire()

                currentTime = time.time()

                #Calculate time since the last time it was called
                #if (self.debug & MODULE_CLIENT_UDP):
                #    logging.debug("Duration: " + str(currentTime - lastTime))

                UDP_MSG = self.getMessage() 
                if UDP_MSG != None:           
                    self.sock.sendto(UDP_MSG, (self.UDP_IP, self.UDP_PORT))

                    if (self.debug & MODULE_CLIENT_UDP):
                        logging.debug(UDP_MSG) 

            except Queue.Empty:
                if (self.debug & MODULE_CLIENT_UDP):
                    logging.debug("Queue Empty")
                pass  

            finally:
                lastTime = currentTime
                self._lock.release()
    
    #Override method  
    def join(self, timeout=None):
        #Stop the thread and wait for it to end        
        self._stopEvent.set()
        self.sock.close()
        threading.Thread.join(self, timeout=timeout)

    def getMessage(self, timeout=2):
        return self._workQueue.get(timeout=timeout)

    def putMessage(self, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full(): 
            self._workQueue.put(msg)   

LP = 1.0

def main():
    try:
        clientThread = UDP_ClientThread(name="Thread-UDP-Client")
        clientThread.daemon = True
        clientThread.start()

        i = 0.0
        while True:            
            udpMsg = str(datetime.datetime.now()) + "," + \
                     str(i) + "," + \
                     str(i) + "#"          
            i += 1.0

            clientThread.putMessage(udpMsg)
            time.sleep(LP)

    except KeyboardInterrupt:  
        clientThread.join()       
        print "Quit" 

if __name__ == "__main__":
    main()
