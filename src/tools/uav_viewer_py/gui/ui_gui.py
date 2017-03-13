# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_gui.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(790, 550)
        MainWindow.setMinimumSize(QtCore.QSize(790, 550))
        MainWindow.setMaximumSize(QtCore.QSize(790, 550))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.takeoffButton = QtWidgets.QPushButton(self.centralwidget)
        self.takeoffButton.setGeometry(QtCore.QRect(80, 380, 220, 41))
        self.takeoffButton.setObjectName("takeoffButton")

        self.playButton = QtWidgets.QPushButton(self.centralwidget)
        self.playButton.setGeometry(QtCore.QRect(80, 425, 100, 51))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playButton.setIcon(icon)
        self.playButton.setObjectName("playButton")
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(200, 425, 100, 51))
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.stopButton.setIcon(icon1)
        self.stopButton.setObjectName("stopButton")

        self.windowsLabel = QtWidgets.QLabel(self.centralwidget)
        self.windowsLabel.setGeometry(QtCore.QRect(350, 380, 71, 21))
        self.windowsLabel.setObjectName("windowsLabel")
        self.cameraCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.cameraCheck.setGeometry(QtCore.QRect(350, 410, 94, 26))
        self.cameraCheck.setObjectName("cameraCheck")
        self.sensorsCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.sensorsCheck.setGeometry(QtCore.QRect(350, 440, 94, 26))
        self.sensorsCheck.setObjectName("sensorsCheck")
        self.colorFilterCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.colorFilterCheck.setGeometry(QtCore.QRect(350, 470, 94, 26))
        self.colorFilterCheck.setObjectName("colorFilterCheck")

        self.altdLabel = QtWidgets.QLabel(self.centralwidget)
        self.altdLabel.setGeometry(QtCore.QRect(220, 340, 61, 21))
        self.altdLabel.setObjectName("altdLabel")

        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 30, 361, 301))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.tlLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.tlLayout.setObjectName("tlLayout")
        ####################NUEVO####################
        self.verticalLayoutWidget_1 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_1.setGeometry(QtCore.QRect(400, 30, 361, 301))
        self.verticalLayoutWidget_1.setObjectName("verticalLayoutWidget_1")
        self.tlLayout_1 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_1)
        self.tlLayout_1.setObjectName("tlLayout_1")
        #######################

        self.rotationLabel = QtWidgets.QLabel(self.centralwidget)
        self.rotationLabel.setGeometry(QtCore.QRect(350, 340, 65, 21))
        self.rotationLabel.setObjectName("rotationLabel")

        self.XLabel = QtWidgets.QLabel(self.centralwidget)
        self.XLabel.setGeometry(QtCore.QRect(20, 340, 21, 21))
        self.XLabel.setObjectName("XLabel")
        self.YLabel = QtWidgets.QLabel(self.centralwidget)
        self.YLabel.setGeometry(QtCore.QRect(130, 340, 21, 21))
        self.YLabel.setObjectName("YLabel")
        self.XValue = QtWidgets.QLabel(self.centralwidget)
        self.XValue.setGeometry(QtCore.QRect(40, 340, 41, 21))
        self.XValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.XValue.setObjectName("XValue")
        self.YValue = QtWidgets.QLabel(self.centralwidget)
        self.YValue.setGeometry(QtCore.QRect(150, 340, 41, 21))
        self.YValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.YValue.setObjectName("YValue")
        self.altdValue = QtWidgets.QLabel(self.centralwidget)
        self.altdValue.setGeometry(QtCore.QRect(280, 340, 41, 21))
        self.altdValue.setAlignment(QtCore.Qt.AlignCenter)
        self.altdValue.setObjectName("altdValue")
        self.rotValue = QtWidgets.QLabel(self.centralwidget)
        self.rotValue.setGeometry(QtCore.QRect(430, 340, 41, 21))
        self.rotValue.setAlignment(QtCore.Qt.AlignCenter)
        self.rotValue.setObjectName("rotValue")

        self.resetButton = QtWidgets.QPushButton(self.centralwidget)
        self.resetButton.setGeometry(QtCore.QRect(80, 480, 220, 41))
        self.resetButton.setObjectName("resetButton")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(480, 380, 150, 150))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.logoLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.logoLayout.setSpacing(0)
        self.logoLayout.setObjectName("logoLayout")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "UAV Viewer"))
        self.takeoffButton.setText(_translate("MainWindow", "Take off"))
        self.playButton.setText(_translate("MainWindow", "Play"))
        self.stopButton.setText(_translate("MainWindow", "Stop"))
        self.windowsLabel.setText(_translate("MainWindow", "Windows:"))
        self.cameraCheck.setText(_translate("MainWindow", "Camera"))
        self.sensorsCheck.setText(_translate("MainWindow", "Sensors"))
        self.colorFilterCheck.setText(_translate("MainWindow", "Color filter"))
        self.altdLabel.setText(_translate("MainWindow", "Altitude:"))
        self.rotationLabel.setText(_translate("MainWindow", "Rotation:"))
        self.XLabel.setText(_translate("MainWindow", "X:"))
        self.YLabel.setText(_translate("MainWindow", "Y:"))
        self.XValue.setText(_translate("MainWindow", "0"))
        self.YValue.setText(_translate("MainWindow", "0"))
        self.altdValue.setText(_translate("MainWindow", "0"))
        self.rotValue.setText(_translate("MainWindow", "0"))
        self.resetButton.setText(_translate("MainWindow", "Reset"))

import resources_rc
