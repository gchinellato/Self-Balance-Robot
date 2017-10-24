#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Description: Log File API
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

import threading
import datetime
import time
import Queue as queue
from constants import *
from Utils.traces.trace import *
from Utils.constants import *

class LogFileThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=queue.Queue(), debug=0):
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

        self.file = None

        logging.info("Log File Module initialized")

    #Override method
    def run(self):
        logging.info("Log File Thread Started")

        lastTime = 0.0
        self.file = open("data.txt", "w")

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                currentTime = time.time()

                msg = self.getMessage()

                #write file
                self.file.write(str(msg))
                self.file.write("\n")
            except queue.Empty:
                pass
            finally:
                lastTime = currentTime

    #Override method
    def join(self, timeout=2):
        #Stop the thread and wait for it to end
        logging.info("Killing Log File Thread...")
        self._stopEvent.set()
        self.file.flush()
        self.file.close()
        threading.Thread.join(self, timeout=timeout)

    def getMessage(self, timeout=2):
        return self._workQueue.get(timeout=timeout)

    def putMessage(self, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full(): 
            self._workQueue.put(msg)   

