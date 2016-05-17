# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainWindow.ui'
#
# Created: Tue Mar  8 22:26:52 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(952, 599)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.groupBox_Accel = QtGui.QGroupBox(self.centralwidget)
        self.groupBox_Accel.setGeometry(QtCore.QRect(20, 20, 291, 141))
        self.groupBox_Accel.setMouseTracking(False)
        self.groupBox_Accel.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.groupBox_Accel.setAutoFillBackground(True)
        self.groupBox_Accel.setObjectName(_fromUtf8("groupBox_Accel"))
        self.label_AccRawX = QtGui.QLabel(self.groupBox_Accel)
        self.label_AccRawX.setGeometry(QtCore.QRect(0, 30, 66, 17))
        self.label_AccRawX.setObjectName(_fromUtf8("label_AccRawX"))
        self.label_AccRawY = QtGui.QLabel(self.groupBox_Accel)
        self.label_AccRawY.setGeometry(QtCore.QRect(0, 60, 66, 17))
        self.label_AccRawY.setObjectName(_fromUtf8("label_AccRawY"))
        self.label_AccRawZ = QtGui.QLabel(self.groupBox_Accel)
        self.label_AccRawZ.setGeometry(QtCore.QRect(0, 90, 66, 17))
        self.label_AccRawZ.setObjectName(_fromUtf8("label_AccRawZ"))
        self.lineEdit_AccRawX = QtGui.QLineEdit(self.groupBox_Accel)
        self.lineEdit_AccRawX.setGeometry(QtCore.QRect(70, 20, 113, 27))
        self.lineEdit_AccRawX.setObjectName(_fromUtf8("lineEdit_AccRawX"))
        self.lineEdit_AccRawY = QtGui.QLineEdit(self.groupBox_Accel)
        self.lineEdit_AccRawY.setGeometry(QtCore.QRect(70, 50, 113, 27))
        self.lineEdit_AccRawY.setObjectName(_fromUtf8("lineEdit_AccRawY"))
        self.lineEdit_AccRawZ = QtGui.QLineEdit(self.groupBox_Accel)
        self.lineEdit_AccRawZ.setGeometry(QtCore.QRect(70, 80, 113, 27))
        self.lineEdit_AccRawZ.setObjectName(_fromUtf8("lineEdit_AccRawZ"))
        self.widgetMpl = QtGui.QWidget(self.centralwidget)
        self.widgetMpl.setGeometry(QtCore.QRect(310, 40, 461, 391))
        self.widgetMpl.setObjectName(_fromUtf8("widgetMpl"))
        self.splitter = QtGui.QSplitter(self.centralwidget)
        self.splitter.setGeometry(QtCore.QRect(20, 160, 148, 81))
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName(_fromUtf8("splitter"))
        self.pushButton = QtGui.QPushButton(self.splitter)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.pushButton_EnableUDPServer = QtGui.QPushButton(self.splitter)
        self.pushButton_EnableUDPServer.setObjectName(_fromUtf8("pushButton_EnableUDPServer"))
        self.pushButton_DisableUDPServer = QtGui.QPushButton(self.splitter)
        self.pushButton_DisableUDPServer.setObjectName(_fromUtf8("pushButton_DisableUDPServer"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 952, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.groupBox_Accel.setTitle(_translate("MainWindow", "Accelerometer", None))
        self.label_AccRawX.setText(_translate("MainWindow", "AccRawX:", None))
        self.label_AccRawY.setText(_translate("MainWindow", "AccRawY:", None))
        self.label_AccRawZ.setText(_translate("MainWindow", "AccRawZ:", None))
        self.pushButton.setText(_translate("MainWindow", "Update", None))
        self.pushButton_EnableUDPServer.setText(_translate("MainWindow", "Enable UDP Server", None))
        self.pushButton_DisableUDPServer.setText(_translate("MainWindow", "Disable UDP Server", None))

