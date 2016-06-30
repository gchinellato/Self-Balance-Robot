#!/usr/bin/python
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
import datetime
import time
import Queue

class Worker(QtCore.QThread):
    def __init__(self, parent = None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent
        self.stop = False
        
    def run(self):
        while not self.stop:
            msg = self.parent.serverUDP.getMessage()
            self.parent.ui.lineEdit_roll.setText(msg[1][1])
            self.parent.ui.lineEdit_pitch.setText(msg[1][2])
            self.parent.ui.lineEdit_yaw.setText(msg[1][3]) 
            self.parent.ui.lineEdit_speedPID.setText(msg[1][4]) 
            self.parent.ui.lineEdit_speedRun.setText(msg[1][5]) 
            self.parent.ui.lineEdit_speedTurn.setText(msg[1][6])  
            self.parent.ui.lineEdit_speedLeft.setText(msg[1][7])
            self.parent.ui.lineEdit_speedRight.setText(msg[1][8])
            time.sleep(0.1)

class mainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QMainWindow.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        #Button actions
        self.ui.pushButton_serverEnable.clicked.connect(self.pushButton_serverEnable_onClicked)  
        self.ui.pushButton_clientEnable.clicked.connect(self.pushButton_clientEnable_onClicked) 
        self.ui.pushButton_sendPID.clicked.connect(self.pushButton_sendPID_onClicked) 
        self.ui.pushButton_sendPID_Zero.clicked.connect(self.pushButton_sendPID_Zero_onClicked) 
        self.ui.pushButton_killUDP.clicked.connect(self.pushButton_killUDP_onClicked) 
        
        self.threads = [] 
        
        self.serverUDPQueue = Queue.Queue(4)
        
        self.worker = Worker(self) 
        
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
        Kp = self.ui.doubleSpinBox_Kp.value()
        Ki = self.ui.doubleSpinBox_Ki.value()
        Kd = self.ui.doubleSpinBox_Kd.value()
        msg = str(datetime.datetime.now()) + "," + \
                  "PID" + "," + \
                  str(Kp) + "," + \
                  str(Ki) + "," + \
                  str(Kd) + "#"
        self.clientUDP.putMessage(msg)
        
    def pushButton_killUDP_onClicked(self): 
        print "TO DO"
        
    def pushButton_sendPID_Zero_onClicked(self): 
        Kp = 0.0
        Ki = 0.0
        Kd = 0.0
        msg = str(datetime.datetime.now()) + "," + \
                  "PID" + "," + \
                  str(Kp) + "," + \
                  str(Ki) + "," + \
                  str(Kd) + "#"
        self.clientUDP.putMessage(msg)
        
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = mainWindow()
    myapp.show()            
    sys.exit(app.exec_())
    

 
    
