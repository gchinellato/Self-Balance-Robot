#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+                       
* @Description: GY80 API - Orientation sensor via I2C bus
                Graphic to check Euler Angles deviation
* @Owner: Guilherme Chinellato                   
* @Email: guilhermechinellato@gmail.com                              
*************************************************
"""

import time
import Queue
import logging
import matplotlib.pyplot as plt
import numpy as np
from Comm.UDP.UDP_Server import UDP_ServerThread

serverUDPQueue = Queue.Queue(8)
serverThread = UDP_ServerThread(name="Thread-UDP-Server", queue=serverUDPQueue)
serverThread.start()

try:
    plt.ion()
    ydata = [0] * 50
    ax1=plt.axes()
    line, = plt.plot(ydata)
    plt.ylim([10,40])

    lastTime = 0.0
    LP = 0.1
    while True:
        try:
            currentTime = time.time()

            #Calculate time since the last time it was called
            logging.debug("Duration: " + str(currentTime - lastTime)) # ~0.035s


            #(timestamp),(data1),(data2)(#)

            PID = float(1)

            #Max and min values
            ymin = float(min(ydata))-10
            ymax = float(max(ydata))+10
            #ymin = -100.0-10
            #ymax = 100.0+10

            plt.ylim([ymin,ymax])
            ydata.append(1.0)
            del ydata[0]
            line.set_xdata(np.arange(len(ydata)))
            line.set_ydata(ydata)  # update the data
            plt.draw() # update the plot  
        except Queue.Empty:
            logging.warning("Queue Empty")
            pass
        finally:
            lastTime = currentTime
            plt.pause(LP)
finally: 
    logging.info("Exiting...")  
    serverThread.join()

