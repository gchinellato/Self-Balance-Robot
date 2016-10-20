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
import matplotlib.pyplot as plt
import numpy as np
from PyQt5 import QtGui, QtCore, QtWidgets

class Plot(QtCore.QThread):
    def __init__(self, parent = None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent
        self.stop = False

    def run(self):
        plt.ion()
        ydata1 = [0] * 50
        ydata2 = [0] * 50
        ax1=plt.axes()
        line1, = plt.plot(ydata1)
        line2, = plt.plot(ydata2)
        plt.ylim([10,40])

        LP = 0.1

        while not self.stop:
            speedPID = float(self.parent.ui.lineEdit_pid_out_speed.text())
            anglePID = float(self.parent.ui.lineEdit_pid_out_angle.text())

            #Max and min values
            #ymin = float(min(ydata1))-10
            #ymax = float(max(ydata1))+10
            ymin = -100.0-10
            ymax = 100.0+10

            plt.ylim([ymin,ymax])
            ydata1.append(anglePID)
            ydata2.append(speedPID)
            del ydata1[0]
            del ydata2[0]
            line1.set_xdata(np.arange(len(ydata1)))
            line2.set_xdata(np.arange(len(ydata2)))

            if self.parent.ui.checkBox_angle_out.checkState() == 2:
                line1.set_ydata(ydata1)  # update the data

            if self.parent.ui.checkBox_speed_out.checkState() == 2:
                line2.set_ydata(ydata2)  # update the data

            plt.draw() # update the plot
            plt.pause(LP)
