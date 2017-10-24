#!/usr/bin/python3

import time
import queue
import logging
import matplotlib.pyplot as plt
import numpy as np
import threading
from PyQt5 import QtGui, QtCore, QtWidgets
import random

class Plot(QtCore.QThread):
    def __init__(self, parent = None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent
        self.stop = False

        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.1
        
    def run(self):
        plt.ion()
        ydata1 = [0] * 50
        ydata2 = [0] * 50
        ydata3 = [0] * 50
        ax1=plt.axes()
        line, = plt.plot(ydata1)
        line2, = plt.plot(ydata2)
        line3, = plt.plot(ydata3)
        plt.ylim([10,40])

        while not self._stopEvent.wait(self._sleepPeriod): 
            timestamp = float(self.parent.ui.lineEdit_timestamp.text())
            setpoint = float(self.parent.ui.lineEdit_dt.text())
            CFRoll = float(self.parent.ui.lineEdit_roll.text())
            CFPitch = float(self.parent.ui.lineEdit_pitch.text())
            CFYaw = float(self.parent.ui.lineEdit_yaw.text())
            PID = float(self.parent.ui.lineEdit_pid_out_angle.text())
            
            #CFPitch = random.randrange(0,80,1)
            #setpoint = random.randrange(0,80,1)
            #PID = random.randrange(0,80,1)

            #Max and min values
            ymin = float(min(ydata1))-10
            ymax = float(max(ydata1))+10
            #ymin = -100.0-10
            #ymax = 100.0+10

            plt.ylim([ymin,ymax])
            ydata1.append(CFPitch)
            ydata2.append(setpoint)
            ydata3.append(PID)
            del ydata1[0]
            del ydata2[0]
            del ydata3[0]
            line.set_xdata(np.arange(len(ydata1)))
            line2.set_xdata(np.arange(len(ydata2)))
            line3.set_xdata(np.arange(len(ydata3)))
            line.set_ydata(ydata1)  # update the data
            line2.set_ydata(ydata2)  # update the data
            line3.set_ydata(ydata3)  # update the data
            plt.draw() # update the plot  

    def join(self):
        self._stopEvent.set()

