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

		self.timestamp = 0
		self.dt = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.encoderTicks1 = 0
        self.encoderTicks2 = 0
        self.distance1 = 0
        self.distance2 = 0
        self.velocity1 = 0
        self.velocity2 = 0
        self.motorSpeed1 = 0
        self.motorSpeed2 = 0
        self.outputPIDSpeed = 0
        self.outputPIDAngle = 0
        self.direction = 0
        self.steering = 0
        self.panTiltV = 0
        self.panTiltH = 0

    def run(self):
        while not self.stop:
            msg = self.parent.serverUDP.getMessage()

            #[(Thread)][(module),(data1),(data2),(data3),(...)(#)]
            print(msg)
            if msg[0] == SERVER_UDP_NAME:
                if msg[1][0] == CMD_SERIAL:
					self.timestamp = msg[1][1]
					self.dt = msg[1][2]
                    self.roll = msg[1][3]
                    self.pitch = msg[1][4]
                    self.yaw = msg[1][5]
                    self.encoderTicks1 = msg[1][6]
                    self.encoderTicks2 = msg[1][7]
                    self.distance1 = msg[1][8]
                    self.distance2 = msg[1][9]
                    self.velocity1 = msg[1][10]
                    self.velocity2 = msg[1][11]
                    self.motorSpeed1 = msg[1][12]
                    self.motorSpeed2 = msg[1][13]
                    self.outputPIDSpeed = msg[1][14]
                    self.outputPIDAngle = msg[1][15]
                    self.direction = msg[1][16]
                    self.steering = msg[1][17]

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
