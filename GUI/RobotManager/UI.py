#!/usr/bin/python3
"""
*************************************************
* @Project: Self Balance  
* @Platform: Raspberry PI 2 B+ / Ubuntu / Qt                      
* @Description: User Interface - IMU Module
* @Owner: Guilherme Chinellato 
* @Email: guilhermechinellato@gmail.com                                                
*************************************************
"""

import sys
from PyQt4 import QtCore, QtGui
from mainWindow import Ui_MainWindow 
from Comm.UDP.UDP_Server import UDP_ServerThread
from Comm.UDP.UDP_Client import UDP_ClientThread
from Utils.traces.trace import *
import matplotlib.pyplot as plt
import numpy as np
import datetime
import time
import Queue

class Worker(QtCore.QThread):
    def __init__(self, parent = None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent
        self.stop = False
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.speedPID = 0
        self.anglePID = 0
        self.speedRun = 0
        self.speedTurn= 0
        self.speedLeft = 0
        self.speedRight = 0
        self.veloA = 0
        self.veloB = 0
        
    def run(self):
        while not self.stop:
            msg = self.parent.serverUDP.getMessage()
            self.roll = msg[1][1]
            self.pitch = msg[1][2]
            self.yaw = msg[1][3]
            self.speedPID = msg[1][4]
            self.anglePID = msg[1][5]
            self.speedRun = msg[1][6]
            self.speedTurn= msg[1][7]
            self.speedLeft = msg[1][8]
            self.speedRight = msg[1][9]
            self.veloB = msg[1][10]
            
            self.parent.ui.lineEdit_roll.setText(self.roll)
            self.parent.ui.lineEdit_pitch.setText(self.pitch)
            self.parent.ui.lineEdit_yaw.setText(self.yaw) 
            self.parent.ui.lineEdit_speedPID.setText(self.speedPID) 
            self.parent.ui.lineEdit_anglePID.setText(self.anglePID) 
            self.parent.ui.lineEdit_speedRun.setText(self.speedRun) 
            self.parent.ui.lineEdit_speedTurn.setText(self.speedTurn)  
            self.parent.ui.lineEdit_speedLeft.setText(self.speedLeft)
            self.parent.ui.lineEdit_speedRight.setText(self.speedRight)
            self.parent.ui.lineEdit_veloB.setText(self.veloB)
            time.sleep(0.1)
            
class Chart_Worker(QtCore.QThread):
    def __init__(self, parent=None):
        QtCore.QThread.__init__(self, parent=parent)
        self.parent = parent
        print "init thread"
        
    def run(self): 
        print "run thread" 
        plt.ion()
        ydata = [0] * 50
        ax1=plt.axes()
        line, = plt.plot(ydata)
        plt.ylim([10,40])
        
        LP = 0.0
        while True:
            try:
                '''if (self.parent.ui.lineEdit_anglePID.text() == "") or (self.parent.ui.lineEdit_anglePID.text() == None):
                    data = 0
                else:
                    data = float(self.parent.ui.lineEdit_anglePID.text())'''
                if (self.parent.ui.lineEdit_anglePID.text() == "") or (self.parent.ui.lineEdit_anglePID.text() == None):
                    data = 0
                else:
                    data = float(self.parent.ui.lineEdit_anglePID.text())
    
                #Max and min values
                #ymin = float(min(ydata))-10
                #ymax = float(max(ydata))+10
                ymin = -100.0
                ymax = 100.0
    
                plt.ylim([ymin,ymax])
                ydata.append(data)
                del ydata[0]
                line.set_xdata(np.arange(len(ydata)))
                line.set_ydata(ydata)  # update the data
                plt.draw() # update the plot  
            finally:
                time.sleep(LP) 

class mainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        setVerbosity("debug")
        
        #Button actions
        self.ui.pushButton_serverEnable.clicked.connect(self.pushButton_serverEnable_onClicked)  
        self.ui.pushButton_clientEnable.clicked.connect(self.pushButton_clientEnable_onClicked) 
        self.ui.pushButton_sendPID.clicked.connect(self.pushButton_sendPID_onClicked) 
        self.ui.pushButton_sendPID_Zero.clicked.connect(self.pushButton_sendPID_Zero_onClicked) 
        self.ui.pushButton_restartUDP.clicked.connect(self.pushButton_RestartUDP_onClicked) 
        self.ui.pushButton_chart.clicked.connect(self.pushButton_showChart_onClicked) 
        self.ui.pushButton_3dModel.clicked.connect(self.pushButton_show3DModel_onClicked) 
        
        self.threads = []        
        self.serverUDPQueue = Queue.Queue(4)        
        self.worker = Worker(self) 
        self.clientUDP = None
        self.serverUDP = None
        
    def pushButton_serverEnable_onClicked(self):  
        #Create and start UDP server thread
        port = int(self.ui.lineEdit_serverPort.text())
        self.serverUDP = UDP_ServerThread(name="Server-UDP-Thread", queue=self.serverUDPQueue, UDP_PORT=port)
        self.serverUDP.daemon = True
        self.threads.append(self.serverUDP)
        self.serverUDP.start()
        self.worker.start()
                
    def pushButton_clientEnable_onClicked(self):           
        #Create and start UDP client thread
        ip = self.ui.lineEdit_clientIP.text()
        port = int(self.ui.lineEdit_clientPort.text())
        self.clientUDP = UDP_ClientThread(name="Client-UDP-Thread", UDP_IP=ip, UDP_PORT=port)
        self.clientUDP.daemon = True
        self.threads.append(self.clientUDP)
        self.clientUDP.start()
        
    def pushButton_sendPID_onClicked(self): 
        speedKp = self.ui.doubleSpinBox_KpSpeed.value()
        speedKi = self.ui.doubleSpinBox_KiSpeed.value()
        speedKd = self.ui.doubleSpinBox_KdSpeed.value()
        angleKp = self.ui.doubleSpinBox_KpAngle.value()
        angleKi = self.ui.doubleSpinBox_KiAngle.value()
        angleKd = self.ui.doubleSpinBox_KdAngle.value()
        speedSetpoint = self.ui.doubleSpinBox_SetpointSpeed.value()
        angleSetpoint = self.ui.doubleSpinBox_SetpointAngle.value()
        msg = str(datetime.datetime.now()) + "," + \
                  "PID" + "," + \
                  str(speedKp) + "," + \
                  str(speedKi) + "," + \
                  str(speedKd) + "," + \
                  str(angleKp) + "," + \
                  str(angleKi) + "," + \
                  str(angleKd) + "," + \
                  str(speedSetpoint) + "," + \
                  str(angleSetpoint) + "#"
        self.clientUDP.putMessage(msg)
        
    def pushButton_RestartUDP_onClicked(self): 
        if self.clientUDP != None and self.clientUDP.is_alive():
            self.clientUDP.join(timeout=1)
        if self.serverUDP != None and self.serverUDP.is_alive():
            self.serverUDP.join(timeout=1)
        print "joined"
        
    def pushButton_sendPID_Zero_onClicked(self): 
        speedKp = 0.0
        speedKi = 0.0
        speedKd = 0.0
        angleKp = 0.0
        angleKi = 0.0
        angleKd = 0.0
        speedSetpoint = 0.0
        angleSetpoint = 0.0
        msg = str(datetime.datetime.now()) + "," + \
                  "PID" + "," + \
                  str(speedKp) + "," + \
                  str(speedKi) + "," + \
                  str(speedKd) + "," + \
                  str(angleKp) + "," + \
                  str(angleKi) + "," + \
                  str(angleKd) + "," + \
                  str(speedSetpoint) + "," + \
                  str(angleSetpoint) + "#"
        self.clientUDP.putMessage(msg)
        
    def pushButton_showChart_onClicked(self): 
        self.worker = Chart_Worker(self)
        self.worker.start()     
        print "start thread"   
        
    def pushButton_show3DModel_onClicked(self): 
        pass   
            
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = mainWindow()
    myapp.show()            
    sys.exit(app.exec_())
    

 
    
