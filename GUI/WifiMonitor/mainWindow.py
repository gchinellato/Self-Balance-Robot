# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWindow.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(620, 407)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setEnabled(True)
        self.tabWidget.setGeometry(QtCore.QRect(0, 10, 611, 341))
        self.tabWidget.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.tabWidget.setAutoFillBackground(False)
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.groupBox = QtWidgets.QGroupBox(self.tab)
        self.groupBox.setGeometry(QtCore.QRect(10, 10, 581, 131))
        self.groupBox.setObjectName("groupBox")
        self.groupBox_2 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_2.setGeometry(QtCore.QRect(0, 20, 191, 111))
        self.groupBox_2.setObjectName("groupBox_2")
        self.label = QtWidgets.QLabel(self.groupBox_2)
        self.label.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.groupBox_2)
        self.label_2.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_2.setObjectName("label_2")
        self.lineEdit_IP_server = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_IP_server.setGeometry(QtCore.QRect(70, 20, 113, 27))
        self.lineEdit_IP_server.setObjectName("lineEdit_IP_server")
        self.lineEdit_port_server = QtWidgets.QLineEdit(self.groupBox_2)
        self.lineEdit_port_server.setGeometry(QtCore.QRect(70, 50, 61, 27))
        self.lineEdit_port_server.setObjectName("lineEdit_port_server")
        self.pushButton_en_server = QtWidgets.QPushButton(self.groupBox_2)
        self.pushButton_en_server.setGeometry(QtCore.QRect(70, 80, 111, 27))
        self.pushButton_en_server.setObjectName("pushButton_en_server")
        self.groupBox_3 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_3.setGeometry(QtCore.QRect(190, 20, 191, 111))
        self.groupBox_3.setObjectName("groupBox_3")
        self.label_3 = QtWidgets.QLabel(self.groupBox_3)
        self.label_3.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.groupBox_3)
        self.label_4.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_4.setObjectName("label_4")
        self.lineEdit_ip_client = QtWidgets.QLineEdit(self.groupBox_3)
        self.lineEdit_ip_client.setGeometry(QtCore.QRect(70, 20, 113, 27))
        self.lineEdit_ip_client.setObjectName("lineEdit_ip_client")
        self.lineEdit_port_client = QtWidgets.QLineEdit(self.groupBox_3)
        self.lineEdit_port_client.setGeometry(QtCore.QRect(70, 50, 61, 27))
        self.lineEdit_port_client.setObjectName("lineEdit_port_client")
        self.pushButton_en_client = QtWidgets.QPushButton(self.groupBox_3)
        self.pushButton_en_client.setGeometry(QtCore.QRect(70, 80, 111, 27))
        self.pushButton_en_client.setObjectName("pushButton_en_client")
        self.groupBox_12 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_12.setGeometry(QtCore.QRect(380, 20, 191, 111))
        self.groupBox_12.setObjectName("groupBox_12")
        self.label_23 = QtWidgets.QLabel(self.groupBox_12)
        self.label_23.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label_23.setObjectName("label_23")
        self.label_24 = QtWidgets.QLabel(self.groupBox_12)
        self.label_24.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_24.setObjectName("label_24")
        self.lineEdit_serial_com = QtWidgets.QLineEdit(self.groupBox_12)
        self.lineEdit_serial_com.setGeometry(QtCore.QRect(70, 20, 113, 27))
        self.lineEdit_serial_com.setObjectName("lineEdit_serial_com")
        self.lineEdit_serial_baud = QtWidgets.QLineEdit(self.groupBox_12)
        self.lineEdit_serial_baud.setGeometry(QtCore.QRect(70, 50, 61, 27))
        self.lineEdit_serial_baud.setObjectName("lineEdit_serial_baud")
        self.pushButton_en_serial = QtWidgets.QPushButton(self.groupBox_12)
        self.pushButton_en_serial.setGeometry(QtCore.QRect(70, 80, 111, 27))
        self.pushButton_en_serial.setObjectName("pushButton_en_serial")
        self.groupBox_4 = QtWidgets.QGroupBox(self.tab)
        self.groupBox_4.setGeometry(QtCore.QRect(20, 159, 241, 111))
        self.groupBox_4.setObjectName("groupBox_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox_4)
        self.label_5.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label_5.setObjectName("label_5")
        self.lineEdit_roll = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_roll.setGeometry(QtCore.QRect(50, 20, 71, 27))
        self.lineEdit_roll.setObjectName("lineEdit_roll")
        self.lineEdit_pitch = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_pitch.setGeometry(QtCore.QRect(50, 50, 71, 27))
        self.lineEdit_pitch.setObjectName("lineEdit_pitch")
        self.label_6 = QtWidgets.QLabel(self.groupBox_4)
        self.label_6.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_6.setObjectName("label_6")
        self.lineEdit_yaw = QtWidgets.QLineEdit(self.groupBox_4)
        self.lineEdit_yaw.setGeometry(QtCore.QRect(50, 80, 71, 27))
        self.lineEdit_yaw.setObjectName("lineEdit_yaw")
        self.label_7 = QtWidgets.QLabel(self.groupBox_4)
        self.label_7.setGeometry(QtCore.QRect(0, 90, 67, 17))
        self.label_7.setObjectName("label_7")
        self.pushButton_3d_model = QtWidgets.QPushButton(self.groupBox_4)
        self.pushButton_3d_model.setGeometry(QtCore.QRect(140, 70, 91, 31))
        self.pushButton_3d_model.setObjectName("pushButton_3d_model")
        self.pushButton_chart_orientation = QtWidgets.QPushButton(self.tab)
        self.pushButton_chart_orientation.setGeometry(QtCore.QRect(160, 180, 91, 31))
        self.pushButton_chart_orientation.setObjectName("pushButton_chart_orientation")
        self.groupBox_5 = QtWidgets.QGroupBox(self.tab)
        self.groupBox_5.setGeometry(QtCore.QRect(270, 160, 321, 141))
        self.groupBox_5.setObjectName("groupBox_5")
        self.label_8 = QtWidgets.QLabel(self.groupBox_5)
        self.label_8.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label_8.setObjectName("label_8")
        self.lineEdit_velocA = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_velocA.setGeometry(QtCore.QRect(70, 20, 71, 27))
        self.lineEdit_velocA.setObjectName("lineEdit_velocA")
        self.lineEdit_velocB = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_velocB.setGeometry(QtCore.QRect(70, 50, 71, 27))
        self.lineEdit_velocB.setObjectName("lineEdit_velocB")
        self.label_9 = QtWidgets.QLabel(self.groupBox_5)
        self.label_9.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_9.setObjectName("label_9")
        self.lineEdit_direction = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_direction.setGeometry(QtCore.QRect(250, 20, 71, 27))
        self.lineEdit_direction.setObjectName("lineEdit_direction")
        self.label_10 = QtWidgets.QLabel(self.groupBox_5)
        self.label_10.setGeometry(QtCore.QRect(180, 30, 67, 17))
        self.label_10.setObjectName("label_10")
        self.label_25 = QtWidgets.QLabel(self.groupBox_5)
        self.label_25.setGeometry(QtCore.QRect(180, 60, 67, 17))
        self.label_25.setObjectName("label_25")
        self.lineEdit_steering = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_steering.setGeometry(QtCore.QRect(250, 50, 71, 27))
        self.lineEdit_steering.setObjectName("lineEdit_steering")
        self.label_11 = QtWidgets.QLabel(self.groupBox_5)
        self.label_11.setGeometry(QtCore.QRect(0, 90, 81, 17))
        self.label_11.setObjectName("label_11")
        self.lineEdit_distanceB = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_distanceB.setGeometry(QtCore.QRect(90, 110, 71, 27))
        self.lineEdit_distanceB.setObjectName("lineEdit_distanceB")
        self.lineEdit_distanceA = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_distanceA.setGeometry(QtCore.QRect(90, 80, 71, 27))
        self.lineEdit_distanceA.setObjectName("lineEdit_distanceA")
        self.label_12 = QtWidgets.QLabel(self.groupBox_5)
        self.label_12.setGeometry(QtCore.QRect(0, 120, 81, 17))
        self.label_12.setObjectName("label_12")
        self.label_28 = QtWidgets.QLabel(self.groupBox_5)
        self.label_28.setGeometry(QtCore.QRect(180, 90, 67, 17))
        self.label_28.setObjectName("label_28")
        self.label_29 = QtWidgets.QLabel(self.groupBox_5)
        self.label_29.setGeometry(QtCore.QRect(180, 120, 67, 17))
        self.label_29.setObjectName("label_29")
        self.lineEdit_panTilt_Vert = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_panTilt_Vert.setGeometry(QtCore.QRect(250, 110, 71, 27))
        self.lineEdit_panTilt_Vert.setObjectName("lineEdit_panTilt_Vert")
        self.lineEdit_panTilt_Horiz = QtWidgets.QLineEdit(self.groupBox_5)
        self.lineEdit_panTilt_Horiz.setGeometry(QtCore.QRect(250, 80, 71, 27))
        self.lineEdit_panTilt_Horiz.setObjectName("lineEdit_panTilt_Horiz")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.groupBox_7 = QtWidgets.QGroupBox(self.tab_2)
        self.groupBox_7.setGeometry(QtCore.QRect(10, 10, 541, 151))
        self.groupBox_7.setObjectName("groupBox_7")
        self.label_16 = QtWidgets.QLabel(self.groupBox_7)
        self.label_16.setGeometry(QtCore.QRect(310, 60, 141, 17))
        self.label_16.setObjectName("label_16")
        self.doubleSpinBox_angle_setpoint = QtWidgets.QDoubleSpinBox(self.groupBox_7)
        self.doubleSpinBox_angle_setpoint.setGeometry(QtCore.QRect(450, 50, 69, 27))
        self.doubleSpinBox_angle_setpoint.setMinimum(-100.0)
        self.doubleSpinBox_angle_setpoint.setSingleStep(0.5)
        self.doubleSpinBox_angle_setpoint.setObjectName("doubleSpinBox_angle_setpoint")
        self.pushButton_angle_zero = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_angle_zero.setGeometry(QtCore.QRect(460, 120, 61, 27))
        self.pushButton_angle_zero.setObjectName("pushButton_angle_zero")
        self.pushButton_angle_set = QtWidgets.QPushButton(self.groupBox_7)
        self.pushButton_angle_set.setGeometry(QtCore.QRect(390, 120, 61, 27))
        self.pushButton_angle_set.setObjectName("pushButton_angle_set")
        self.groupBox_11 = QtWidgets.QGroupBox(self.groupBox_7)
        self.groupBox_11.setGeometry(QtCore.QRect(150, 20, 151, 121))
        self.groupBox_11.setObjectName("groupBox_11")
        self.doubleSpinBox_angle_ki_Aggr = QtWidgets.QDoubleSpinBox(self.groupBox_11)
        self.doubleSpinBox_angle_ki_Aggr.setGeometry(QtCore.QRect(70, 60, 69, 27))
        self.doubleSpinBox_angle_ki_Aggr.setSingleStep(0.1)
        self.doubleSpinBox_angle_ki_Aggr.setObjectName("doubleSpinBox_angle_ki_Aggr")
        self.doubleSpinBox_angle_kp_Aggr = QtWidgets.QDoubleSpinBox(self.groupBox_11)
        self.doubleSpinBox_angle_kp_Aggr.setGeometry(QtCore.QRect(70, 30, 69, 27))
        self.doubleSpinBox_angle_kp_Aggr.setDecimals(2)
        self.doubleSpinBox_angle_kp_Aggr.setSingleStep(0.5)
        self.doubleSpinBox_angle_kp_Aggr.setObjectName("doubleSpinBox_angle_kp_Aggr")
        self.doubleSpinBox_angle_kd_Aggr = QtWidgets.QDoubleSpinBox(self.groupBox_11)
        self.doubleSpinBox_angle_kd_Aggr.setGeometry(QtCore.QRect(70, 90, 69, 27))
        self.doubleSpinBox_angle_kd_Aggr.setSingleStep(0.1)
        self.doubleSpinBox_angle_kd_Aggr.setObjectName("doubleSpinBox_angle_kd_Aggr")
        self.label_21 = QtWidgets.QLabel(self.groupBox_11)
        self.label_21.setGeometry(QtCore.QRect(10, 70, 67, 17))
        self.label_21.setObjectName("label_21")
        self.label_22 = QtWidgets.QLabel(self.groupBox_11)
        self.label_22.setGeometry(QtCore.QRect(10, 100, 67, 17))
        self.label_22.setObjectName("label_22")
        self.label_20 = QtWidgets.QLabel(self.groupBox_11)
        self.label_20.setGeometry(QtCore.QRect(10, 40, 67, 17))
        self.label_20.setObjectName("label_20")
        self.groupBox_14 = QtWidgets.QGroupBox(self.groupBox_7)
        self.groupBox_14.setGeometry(QtCore.QRect(0, 20, 151, 131))
        self.groupBox_14.setObjectName("groupBox_14")
        self.doubleSpinBox_angle_kp = QtWidgets.QDoubleSpinBox(self.groupBox_14)
        self.doubleSpinBox_angle_kp.setGeometry(QtCore.QRect(70, 30, 69, 27))
        self.doubleSpinBox_angle_kp.setDecimals(2)
        self.doubleSpinBox_angle_kp.setSingleStep(0.5)
        self.doubleSpinBox_angle_kp.setObjectName("doubleSpinBox_angle_kp")
        self.label_14 = QtWidgets.QLabel(self.groupBox_14)
        self.label_14.setGeometry(QtCore.QRect(10, 70, 67, 17))
        self.label_14.setObjectName("label_14")
        self.doubleSpinBox_angle_kd = QtWidgets.QDoubleSpinBox(self.groupBox_14)
        self.doubleSpinBox_angle_kd.setGeometry(QtCore.QRect(70, 90, 69, 27))
        self.doubleSpinBox_angle_kd.setSingleStep(0.1)
        self.doubleSpinBox_angle_kd.setObjectName("doubleSpinBox_angle_kd")
        self.label_13 = QtWidgets.QLabel(self.groupBox_14)
        self.label_13.setGeometry(QtCore.QRect(10, 40, 67, 17))
        self.label_13.setObjectName("label_13")
        self.doubleSpinBox_angle_ki = QtWidgets.QDoubleSpinBox(self.groupBox_14)
        self.doubleSpinBox_angle_ki.setGeometry(QtCore.QRect(70, 60, 69, 27))
        self.doubleSpinBox_angle_ki.setSingleStep(0.1)
        self.doubleSpinBox_angle_ki.setObjectName("doubleSpinBox_angle_ki")
        self.label_15 = QtWidgets.QLabel(self.groupBox_14)
        self.label_15.setGeometry(QtCore.QRect(10, 100, 67, 17))
        self.label_15.setObjectName("label_15")
        self.doubleSpinBox_angle_max = QtWidgets.QDoubleSpinBox(self.groupBox_7)
        self.doubleSpinBox_angle_max.setGeometry(QtCore.QRect(450, 80, 69, 27))
        self.doubleSpinBox_angle_max.setMinimum(-100.0)
        self.doubleSpinBox_angle_max.setSingleStep(0.5)
        self.doubleSpinBox_angle_max.setObjectName("doubleSpinBox_angle_max")
        self.label_30 = QtWidgets.QLabel(self.groupBox_7)
        self.label_30.setGeometry(QtCore.QRect(310, 90, 141, 17))
        self.label_30.setObjectName("label_30")
        self.groupBox_8 = QtWidgets.QGroupBox(self.tab_2)
        self.groupBox_8.setGeometry(QtCore.QRect(10, 160, 151, 151))
        self.groupBox_8.setObjectName("groupBox_8")
        self.label_17 = QtWidgets.QLabel(self.groupBox_8)
        self.label_17.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label_17.setObjectName("label_17")
        self.label_18 = QtWidgets.QLabel(self.groupBox_8)
        self.label_18.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_18.setObjectName("label_18")
        self.label_19 = QtWidgets.QLabel(self.groupBox_8)
        self.label_19.setGeometry(QtCore.QRect(0, 90, 67, 17))
        self.label_19.setObjectName("label_19")
        self.doubleSpinBox_speed_kp = QtWidgets.QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_kp.setGeometry(QtCore.QRect(40, 20, 69, 27))
        self.doubleSpinBox_speed_kp.setSingleStep(0.1)
        self.doubleSpinBox_speed_kp.setObjectName("doubleSpinBox_speed_kp")
        self.doubleSpinBox_speed_ki = QtWidgets.QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_ki.setGeometry(QtCore.QRect(40, 50, 69, 27))
        self.doubleSpinBox_speed_ki.setSingleStep(0.1)
        self.doubleSpinBox_speed_ki.setObjectName("doubleSpinBox_speed_ki")
        self.doubleSpinBox_speed_kd = QtWidgets.QDoubleSpinBox(self.groupBox_8)
        self.doubleSpinBox_speed_kd.setGeometry(QtCore.QRect(40, 80, 69, 27))
        self.doubleSpinBox_speed_kd.setSingleStep(0.1)
        self.doubleSpinBox_speed_kd.setObjectName("doubleSpinBox_speed_kd")
        self.pushButton_speed_zero = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_speed_zero.setGeometry(QtCore.QRect(80, 110, 61, 27))
        self.pushButton_speed_zero.setObjectName("pushButton_speed_zero")
        self.pushButton_speed_set = QtWidgets.QPushButton(self.groupBox_8)
        self.pushButton_speed_set.setGeometry(QtCore.QRect(2, 110, 61, 27))
        self.pushButton_speed_set.setObjectName("pushButton_speed_set")
        self.groupBox_13 = QtWidgets.QGroupBox(self.tab_2)
        self.groupBox_13.setGeometry(QtCore.QRect(180, 160, 141, 91))
        self.groupBox_13.setObjectName("groupBox_13")
        self.label_26 = QtWidgets.QLabel(self.groupBox_13)
        self.label_26.setGeometry(QtCore.QRect(0, 30, 67, 17))
        self.label_26.setObjectName("label_26")
        self.lineEdit_pid_out_speed = QtWidgets.QLineEdit(self.groupBox_13)
        self.lineEdit_pid_out_speed.setGeometry(QtCore.QRect(70, 20, 71, 27))
        self.lineEdit_pid_out_speed.setObjectName("lineEdit_pid_out_speed")
        self.lineEdit_pid_out_angle = QtWidgets.QLineEdit(self.groupBox_13)
        self.lineEdit_pid_out_angle.setGeometry(QtCore.QRect(70, 50, 71, 27))
        self.lineEdit_pid_out_angle.setObjectName("lineEdit_pid_out_angle")
        self.label_27 = QtWidgets.QLabel(self.groupBox_13)
        self.label_27.setGeometry(QtCore.QRect(0, 60, 67, 17))
        self.label_27.setObjectName("label_27")
        self.groupBox_9 = QtWidgets.QGroupBox(self.tab_2)
        self.groupBox_9.setGeometry(QtCore.QRect(340, 160, 251, 91))
        self.groupBox_9.setObjectName("groupBox_9")
        self.checkBox_angle_out = QtWidgets.QCheckBox(self.groupBox_9)
        self.checkBox_angle_out.setGeometry(QtCore.QRect(0, 30, 121, 22))
        self.checkBox_angle_out.setObjectName("checkBox_angle_out")
        self.checkBox_speed_out = QtWidgets.QCheckBox(self.groupBox_9)
        self.checkBox_speed_out.setGeometry(QtCore.QRect(0, 60, 121, 22))
        self.checkBox_speed_out.setObjectName("checkBox_speed_out")
        self.pushButton_chart_pid = QtWidgets.QPushButton(self.groupBox_9)
        self.pushButton_chart_pid.setGeometry(QtCore.QRect(130, 40, 91, 31))
        self.pushButton_chart_pid.setObjectName("pushButton_chart_pid")
        self.lineEdit_pitch_tab_PID = QtWidgets.QLineEdit(self.tab_2)
        self.lineEdit_pitch_tab_PID.setGeometry(QtCore.QRect(250, 270, 71, 27))
        self.lineEdit_pitch_tab_PID.setObjectName("lineEdit_pitch_tab_PID")
        self.label_31 = QtWidgets.QLabel(self.tab_2)
        self.label_31.setGeometry(QtCore.QRect(180, 280, 67, 17))
        self.label_31.setObjectName("label_31")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.groupBox_10 = QtWidgets.QGroupBox(self.tab_3)
        self.groupBox_10.setGeometry(QtCore.QRect(10, 10, 251, 211))
        self.groupBox_10.setObjectName("groupBox_10")
        self.checkBox_en_arduino = QtWidgets.QCheckBox(self.groupBox_10)
        self.checkBox_en_arduino.setGeometry(QtCore.QRect(0, 30, 141, 22))
        self.checkBox_en_arduino.setObjectName("checkBox_en_arduino")
        self.checkBox_en_cv = QtWidgets.QCheckBox(self.groupBox_10)
        self.checkBox_en_cv.setGeometry(QtCore.QRect(0, 60, 191, 22))
        self.checkBox_en_cv.setObjectName("checkBox_en_cv")
        self.pushButton_control_set = QtWidgets.QPushButton(self.groupBox_10)
        self.pushButton_control_set.setGeometry(QtCore.QRect(150, 170, 91, 31))
        self.pushButton_control_set.setObjectName("pushButton_control_set")
        self.tabWidget.addTab(self.tab_3, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 620, 25))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Wifi Monitor"))
        self.groupBox.setTitle(_translate("MainWindow", "Connection settings"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Server UDP"))
        self.label.setText(_translate("MainWindow", "IP Addr:"))
        self.label_2.setText(_translate("MainWindow", "Port:"))
        self.lineEdit_IP_server.setText(_translate("MainWindow", "192.168.1.35"))
        self.lineEdit_port_server.setText(_translate("MainWindow", "5000"))
        self.pushButton_en_server.setText(_translate("MainWindow", "Enable"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Client UDP"))
        self.label_3.setText(_translate("MainWindow", "IP Addr:"))
        self.label_4.setText(_translate("MainWindow", "Port:"))
        self.lineEdit_ip_client.setText(_translate("MainWindow", "192.168.1.49"))
        self.lineEdit_port_client.setText(_translate("MainWindow", "5001"))
        self.pushButton_en_client.setText(_translate("MainWindow", "Enable"))
        self.groupBox_12.setTitle(_translate("MainWindow", "Serial"))
        self.label_23.setText(_translate("MainWindow", "COM:"))
        self.label_24.setText(_translate("MainWindow", "Baud:"))
        self.lineEdit_serial_com.setText(_translate("MainWindow", "/dev/ttyUSB0"))
        self.lineEdit_serial_baud.setText(_translate("MainWindow", "19200"))
        self.pushButton_en_serial.setText(_translate("MainWindow", "Enable"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Orientation"))
        self.label_5.setText(_translate("MainWindow", "Roll:"))
        self.label_6.setText(_translate("MainWindow", "Pitch"))
        self.label_7.setText(_translate("MainWindow", "Yaw"))
        self.pushButton_3d_model.setText(_translate("MainWindow", "3D Model..."))
        self.pushButton_chart_orientation.setText(_translate("MainWindow", "Chart..."))
        self.groupBox_5.setTitle(_translate("MainWindow", "Motor"))
        self.label_8.setText(_translate("MainWindow", "Velo A:"))
        self.label_9.setText(_translate("MainWindow", "Velo B:"))
        self.label_10.setText(_translate("MainWindow", "Direction:"))
        self.label_25.setText(_translate("MainWindow", "Steering:"))
        self.label_11.setText(_translate("MainWindow", "Distance A:"))
        self.label_12.setText(_translate("MainWindow", "Distance B:"))
        self.label_28.setText(_translate("MainWindow", "Head H:"))
        self.label_29.setText(_translate("MainWindow", "Head V:"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Monitor"))
        self.groupBox_7.setTitle(_translate("MainWindow", "Angle Control"))
        self.label_16.setText(_translate("MainWindow", "Setpoint correction:"))
        self.pushButton_angle_zero.setText(_translate("MainWindow", "Zero"))
        self.pushButton_angle_set.setText(_translate("MainWindow", "Set"))
        self.groupBox_11.setTitle(_translate("MainWindow", "Aggresive mode"))
        self.label_21.setText(_translate("MainWindow", "Ki Aggr:"))
        self.label_22.setText(_translate("MainWindow", "Kd Aggr:"))
        self.label_20.setText(_translate("MainWindow", "Kp Aggr:"))
        self.groupBox_14.setTitle(_translate("MainWindow", "Conservative mode"))
        self.label_14.setText(_translate("MainWindow", "Ki Cons:"))
        self.label_13.setText(_translate("MainWindow", "Kp Cons:"))
        self.label_15.setText(_translate("MainWindow", "Kd Cons:"))
        self.label_30.setText(_translate("MainWindow", "Max Incli. Angle:"))
        self.groupBox_8.setTitle(_translate("MainWindow", "Speed Control"))
        self.label_17.setText(_translate("MainWindow", "Kp:"))
        self.label_18.setText(_translate("MainWindow", "Ki:"))
        self.label_19.setText(_translate("MainWindow", "Kd:"))
        self.pushButton_speed_zero.setText(_translate("MainWindow", "Zero"))
        self.pushButton_speed_set.setText(_translate("MainWindow", "Set"))
        self.groupBox_13.setTitle(_translate("MainWindow", "PID output"))
        self.label_26.setText(_translate("MainWindow", "Speed:"))
        self.label_27.setText(_translate("MainWindow", "Angle:"))
        self.groupBox_9.setTitle(_translate("MainWindow", "Chart"))
        self.checkBox_angle_out.setText(_translate("MainWindow", "Angle output"))
        self.checkBox_speed_out.setText(_translate("MainWindow", "Speed output"))
        self.pushButton_chart_pid.setText(_translate("MainWindow", "Plot"))
        self.label_31.setText(_translate("MainWindow", "Pitch"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "PID"))
        self.groupBox_10.setTitle(_translate("MainWindow", "Commands"))
        self.checkBox_en_arduino.setText(_translate("MainWindow", "Enable Arduino"))
        self.checkBox_en_cv.setText(_translate("MainWindow", "Enable Computer Vision"))
        self.pushButton_control_set.setText(_translate("MainWindow", "Set"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Control"))

