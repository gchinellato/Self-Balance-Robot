#!/usr/bin/python3
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+ / Ubuntu / Qt                      
* @Description:
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import sys
from PyQt5 import QtGui, QtCore, QtWidgets
from Utils.traces.trace import *

import time
        
class Worker(QtCore.QThread):
    def __init__(self, parent = None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent
        self.stop = False
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.speedRun = 0
        self.speedTurn = 0
        self.speedLeft = 0
        self.speedRight = 0
        self.veloc = 0
        self.panTiltV = 0
        self.panTiltH = 0
        
    def run(self):
        while not self.stop:
            msg = self.parent.serverUDP.getMessage()

            if msg[0] == "Server-UDP-Thread": 
                if msg[1][0] == "BALANCE":
                    self.roll = msg[1][2]
                    self.pitch = msg[1][3]
                    self.yaw = msg[1][4]
                    self.speedRun = msg[1][5]
                    self.speedTurn = msg[1][6]
                    self.speedLeft = msg[1][7]
                    self.speedRight = msg[1][8]
                    self.veloc = msg[1][9]
                    
                    self.parent.ui.lineEdit_roll.setText(self.roll)
                    self.parent.ui.lineEdit_pitch.setText(self.pitch)
                    self.parent.ui.lineEdit_yaw.setText(self.yaw) 
                    self.parent.ui.lineEdit_analog1_horiz.setText(self.speedRun) 
                    self.parent.ui.lineEdit_analog1_vertical.setText(self.speedTurn) 
                    self.parent.ui.lineEdit_velocA.setText(self.speedLeft)  
                    self.parent.ui.lineEdit_velocB.setText(self.speedRight)
                    self.parent.ui.lineEdit_veloc_avg.setText(self.veloc)
                    
                if msg[1][0] == "PAN_TILT":
                    self.panTiltV = msg[1][2]
                    self.panTiltH = msg[1][3]
                    
                    self.parent.ui.lineEdit_analog2_horiz.setText(self.panTiltH) 
                    self.parent.ui.lineEdit_analog2_vertical.setText(self.panTiltV) 
            else:
                print("Invalid message")
            time.sleep(0.02)
