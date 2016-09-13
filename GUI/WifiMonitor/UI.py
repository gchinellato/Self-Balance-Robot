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
import pyqtgraph as pg
from mainWindow import Ui_MainWindow 

from UDP.UDP_Server import UDP_ServerThread
from UDP.UDP_Client import UDP_ClientThread
from worker import Worker
from TriDisplay import TriModel
from Utils.traces.trace import *

import datetime
import time
import queue
import numpy as np

class mainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self) 
        
        setVerbosity("debug")
        
        #Button actions
        self.ui.pushButton_en_server.clicked.connect(self.pushButton_serverEnable_onClicked)  
        self.ui.pushButton_en_client.clicked.connect(self.pushButton_clientEnable_onClicked) 
        self.ui.pushButton_chart_orientation.clicked.connect(self.pushButton_chartOrientation_onClicked) 
        self.ui.pushButton_3d_model.clicked.connect(self.pushButton_3D_Model_onClicked) 
        self.ui.pushButton_angle_set.clicked.connect(self.pushButton_angleSetPID_onClicked) 
        self.ui.pushButton_speed_set.clicked.connect(self.pushButton_speedSetPID_onClicked) 
        self.ui.pushButton_angle_zero.clicked.connect(self.pushButton_angleZeroPID_onClicked) 
        self.ui.pushButton_speed_zero.clicked.connect(self.pushButton_speedZeroPID_onClicked) 
        
        self.serverUDPQueue = queue.Queue(4)        
        self.threads = []              
        self.worker = None         
        self.clientUDP = None
        self.serverUDP = None
        
    def pushButton_serverEnable_onClicked(self):  
        #Create and start UDP server thread
        port = int(self.ui.lineEdit_port_server.text())

        if self.serverUDP != None and self.worker != None: 
            self.worker.terminate() 
            self.serverUDP.join(timeout=1)
            
        self.worker = Worker(self)
        self.serverUDP = UDP_ServerThread(name="Server-UDP-Thread", queue=self.serverUDPQueue, UDP_PORT=port)
        self.serverUDP.daemon = True
        self.threads.append(self.serverUDP)
            
        self.serverUDP.start()
        self.worker.start()
                
    def pushButton_clientEnable_onClicked(self):           
        #Create and start UDP client thread
        ip = self.ui.lineEdit_ip_client.text()
        port = int(self.ui.lineEdit_port_client.text())
        
        if self.clientUDP != None:
            self.clientUDP.join(timeout=1)
            
        self.clientUDP = UDP_ClientThread(name="Client-UDP-Thread", UDP_IP=ip, UDP_PORT=port)
        self.clientUDP.daemon = True
        self.threads.append(self.clientUDP)
        self.clientUDP.start()  
        
    def pushButton_chartOrientation_onClicked(self):
        pass 
    
    def pushButton_3D_Model_onClicked(self):
        self.triModel = TriModel(self)
        self.triModel.start()
    
    def pushButton_angleSetPID_onClicked(self):
        angleKpCons = self.ui.doubleSpinBox_angle_kp.value()
        angleKiCons = self.ui.doubleSpinBox_angle_ki.value()
        angleKdCons = self.ui.doubleSpinBox_angle_kd.value()
        
        angleKpAggr = self.ui.doubleSpinBox_angle_kp_Aggr.value()
        angleKiAggr = self.ui.doubleSpinBox_angle_ki_Aggr.value()
        angleKdAggr = self.ui.doubleSpinBox_angle_kd_Aggr.value()
        angleSetpoint = self.ui.doubleSpinBox_angle_setpoint.value()
        
        msg = "SET_PID_ANGLE" + "," + \
              str(datetime.datetime.now()) + "," + \
              str(angleKpCons) + "," + \
              str(angleKiCons) + "," + \
              str(angleKdCons) + "," + \
              str(angleKpAggr) + "," + \
              str(angleKiAggr) + "," + \
              str(angleKdAggr) + "," + \
              str(angleSetpoint) + "#"
        
        # Sending UDP packets...
        if (self.clientUDP != None):
            self.clientUDP.putMessage(msg)
    
    def pushButton_speedSetPID_onClicked(self):
        speedKpCons = self.ui.doubleSpinBox_speed_kp.value()
        speedKiCons = self.ui.doubleSpinBox_speed_ki.value()
        speedKdCons = self.ui.doubleSpinBox_speed_kd.value()
        
        msg = "SET_PID_SPEED" + "," + \
              str(datetime.datetime.now()) + "," + \
              str(speedKpCons) + "," + \
              str(speedKiCons) + "," + \
              str(speedKdCons) + "#"
        
        # Sending UDP packets...
        if (self.clientUDP != None):
            self.clientUDP.putMessage(msg)
    
    def pushButton_angleZeroPID_onClicked(self):
        msg = "SET_PID_ANGLE" + "," + \
              str(datetime.datetime.now()) + "," + \
              str("0") + "," + \
              str("0") + "," + \
              str("0") + "," + \
              str("0") + "," + \
              str("0") + "," + \
              str("0") + "," + \
              str("0") + "#"
        
        # Sending UDP packets...
        if (self.clientUDP != None):
            self.clientUDP.putMessage(msg)
    
    def pushButton_speedZeroPID_onClicked(self):
        msg = "SET_PID_SPEED" + "," + \
              str(datetime.datetime.now()) + "," + \
              str("0") + "," + \
              str("0") + "," + \
              str("0") + "#"
        
        # Sending UDP packets...
        if (self.clientUDP != None):
            self.clientUDP.putMessage(msg)
             
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    myapp = mainWindow()
    myapp.show()            
    sys.exit(app.exec_())

    

 
    
