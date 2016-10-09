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
from constants import *

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

            #[(Thread)][(module),(data1),(data2),(data3),(...)(#)]
            print(msg)
            if msg[0] == SERVER_UDP_NAME:
                if msg[1][0] == CMD_SERIAL:
                    self.roll = msg[1][1]
                    self.pitch = msg[1][2]
                    self.yaw = msg[1][3]
                    self.encoderTicks1 = msg[1][4]
                    self.encoderTicks2 = msg[1][5]
                    self.distance1 = msg[1][6]
                    self.distance2 = msg[1][7]
                    self.velocity1 = msg[1][8]
                    self.velocity2 = msg[1][9]
                    self.outputPIDSpeed = msg[1][10]
                    self.outputPIDAngle = msg[1][11]
                    self.direction = msg[1][12]
                    self.steering = msg[1][13]

                    self.parent.ui.lineEdit_roll.setText(self.roll)
                    self.parent.ui.lineEdit_pitch.setText(self.pitch)
                    self.parent.ui.lineEdit_pitch_tab_PID.setText(self.pitch)
                    self.parent.ui.lineEdit_yaw.setText(self.yaw)
                    self.parent.ui.lineEdit_distanceA.setText(self.distance1)
                    self.parent.ui.lineEdit_distanceB.setText(self.distance2)
                    self.parent.ui.lineEdit_velocA.setText(self.velocity1)
                    self.parent.ui.lineEdit_velocB.setText(self.velocity2)
                    self.parent.ui.lineEdit_direction.setText(self.direction)
                    self.parent.ui.lineEdit_steering.setText(self.steering)
                    self.parent.ui.lineEdit_pid_out_speed.setText(self.outputPIDSpeed)
                    self.parent.ui.lineEdit_pid_out_angle.setText(self.outputPIDAngle)

                if msg[1][0] == CMD_PAN_TILT:
                    self.panTiltV = msg[1][1]
                    self.panTiltH = msg[1][2]

                    self.parent.ui.lineEdit_panTilt_Horiz.setText(self.panTiltH)
                    self.parent.ui.lineEdit_panTilt_Vert.setText(self.panTiltV)
            else:
                print("Invalid message")
            time.sleep(0.02)
