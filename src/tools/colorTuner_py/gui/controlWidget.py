#
#  Copyright (C) 1997-2015 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#

from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtWidgets import QWidget, QLabel, QRadioButton, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy, QSlider


class ControlWidget(QWidget):
    
    #controlUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ControlWidget, self).__init__()
        self.winParent=winParent
        self.initUI()
        
    def initUI(self):

        self.setMinimumSize(1340,200)
        self.setMaximumSize(1340,200)

        self.origButton = QRadioButton("Original")
        self.hsvButton = QRadioButton("HSV")

        self.origButton.toggled.connect(lambda:self.origState())       
        self.hsvButton.toggled.connect(lambda:self.hsvState())

        self.origButton.setChecked(True)

        self.hLayout = QHBoxLayout(self)
        self.hLayout.setObjectName("hLayout")

        ''' Vertical Layout for radio buttons '''
        self.radioLayout = QVBoxLayout()
        self.radioLayout.setObjectName("radioLayout")
        self.radioLayout.addWidget(self.origButton)
        self.radioLayout.addWidget(self.hsvButton)
        self.vSpacer = QSpacerItem(30, 500, QSizePolicy.Ignored, QSizePolicy.Ignored);
        self.radioLayout.addItem(self.vSpacer)


        ''' Vertical Layout for HMIN Slider''' 
        self.hminLayout = QVBoxLayout()
        self.hminLayout.setObjectName("hminLayout")
        self.hminLabel = QLabel("HMin")
        self.hminValue = QLabel("0")
        self.hminValue.setAlignment(Qt.AlignCenter);
        self.hminSlider = QSlider(Qt.Vertical)
        self.hminSlider.setMinimum(0)
        self.hminSlider.setMaximum(600)
        self.hminSlider.setValue(0)
        self.hminLayout.addWidget(self.hminLabel)
        self.hminLayout.addWidget(self.hminValue)
        self.hminLayout.addWidget(self.hminSlider)

        ''' Vertical Layout for HMAX Slider''' 
        self.hmaxLayout = QVBoxLayout()
        self.hmaxLayout.setObjectName("hmaxLayout")
        self.hmaxLabel = QLabel("HMax")
        self.hmaxValue = QLabel("6")
        self.hmaxValue.setAlignment(Qt.AlignCenter);
        self.hmaxSlider = QSlider(Qt.Vertical)
        self.hmaxSlider.setMinimum(0)
        self.hmaxSlider.setMaximum(6)
        self.hmaxSlider.setValue(6)
        self.hmaxLayout.addWidget(self.hmaxLabel)
        self.hmaxLayout.addWidget(self.hmaxValue)
        self.hmaxLayout.addWidget(self.hmaxSlider)

        ''' Vertical Layout for SMIN Slider'''
        self.sminLayout = QVBoxLayout()
        self.sminLayout.setObjectName("sminLayout")
        self.sminLabel = QLabel("SMin")
        self.sminValue = QLabel("0")
        self.sminValue.setAlignment(Qt.AlignCenter);
        self.sminSlider = QSlider(Qt.Vertical)
        self.sminSlider.setMinimum(0)
        self.sminSlider.setMaximum(1)
        self.sminSlider.setValue(0)
        self.sminLayout.addWidget(self.sminLabel)
        self.sminLayout.addWidget(self.sminValue)
        self.sminLayout.addWidget(self.sminSlider)

        ''' Vertical Layout for SMAX Slider'''
        self.smaxLayout = QVBoxLayout()
        self.smaxLayout.setObjectName("smaxLayout")
        self.smaxLabel = QLabel("SMax")
        self.smaxValue = QLabel("1")
        self.smaxValue.setAlignment(Qt.AlignCenter);
        self.smaxSlider = QSlider(Qt.Vertical)
        self.smaxSlider.setMinimum(0)
        self.smaxSlider.setMaximum(1)
        self.smaxSlider.setValue(1)
        self.smaxLayout.addWidget(self.smaxLabel)
        self.smaxLayout.addWidget(self.smaxValue)
        self.smaxLayout.addWidget(self.smaxSlider)

        ''' Vertical Layout for VMIN Slider'''
        self.vminLayout = QVBoxLayout()
        self.vminLayout.setObjectName("vminLayout")
        self.vminLabel = QLabel("VMin")
        self.vminValue = QLabel("0")
        self.vminValue.setAlignment(Qt.AlignCenter);
        self.vminSlider = QSlider(Qt.Vertical)
        self.vminSlider.setMinimum(0)
        self.vminSlider.setMaximum(255)
        self.vminSlider.setValue(0)
        self.vminLayout.addWidget(self.vminLabel)
        self.vminLayout.addWidget(self.vminValue)
        self.vminLayout.addWidget(self.vminSlider)

        ''' Vertical Layout for SMAX Slider'''
        self.vmaxLayout = QVBoxLayout()
        self.vmaxLayout.setObjectName("vmaxLayout")
        self.vmaxLabel = QLabel("VMax")
        self.vmaxValue = QLabel("255")
        self.vmaxValue.setAlignment(Qt.AlignCenter);
        self.vmaxSlider = QSlider(Qt.Vertical)
        self.vmaxSlider.setMinimum(0)
        self.vmaxSlider.setMaximum(255)
        self.vmaxSlider.setValue(255)
        self.vmaxLayout.addWidget(self.vmaxLabel)
        self.vmaxLayout.addWidget(self.vmaxValue)
        self.vmaxLayout.addWidget(self.vmaxSlider)
    
        self.hLayout.addLayout(self.radioLayout)
        self.hLayout.addLayout(self.hminLayout)
        self.hLayout.addLayout(self.hmaxLayout)
        self.hLayout.addLayout(self.sminLayout)
        self.hLayout.addLayout(self.smaxLayout)
        self.hLayout.addLayout(self.vminLayout)
        self.hLayout.addLayout(self.vmaxLayout)
        self.setLayout(self.hLayout)

        self.hminSlider.valueChanged.connect(self.changeHmin)     

        

    def origState(self):
        if self.origButton.isChecked():
            self.hsvButton.setChecked(False)


    def hsvState(self):
        if self.hsvButton.isChecked():
            self.origButton.setChecked(False)

    def changeHmin(self):
        value = self.hminSlider.value() / 100.0
        self.hminValue.setText(str(value))

           
    def closeEvent(self, event):
        self.winParent.closeimagesWidget()