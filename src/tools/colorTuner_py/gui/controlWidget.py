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
from  sensors.cameraFilter import RGBLimits, YUVLimits, HSVLimits



class ControlWidget(QWidget):
    
    #controlUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ControlWidget, self).__init__()
        self.winParent=winParent
        self.initUI()
        self.rgbLimits = RGBLimits()
        self.yuvLimits = YUVLimits()
        self.hsvLimits = HSVLimits()
        
    def initUI(self):

        self.setMinimumSize(1340,200)
        self.setMaximumSize(1340,200)

        '''Radio buttons for Original/RGB/HSV/YUV images'''
        self.origButton = QRadioButton("Original")
        self.rgbButton = QRadioButton("RGB")
        self.hsvButton = QRadioButton("HSV")
        self.yuvButton = QRadioButton("YUV")

        '''Signals for toggled radio buttons'''
        self.origButton.toggled.connect(lambda:self.origButtonState())       
        self.rgbButton.toggled.connect(lambda:self.rgbButtonState())
        self.hsvButton.toggled.connect(lambda:self.hsvButtonState())
        self.yuvButton.toggled.connect(lambda:self.yuvButtonState())

        self.origButton.setChecked(True)

        '''Main layout of the widget will contain several vertical layouts'''
        self.hLayout = QHBoxLayout(self)
        self.hLayout.setObjectName("hLayout")

        ''' Vertical Layout for radio buttons '''
        self.radioLayout = QVBoxLayout()
        self.radioLayout.setObjectName("radioLayout")
        self.radioLayout.addWidget(self.origButton)
        self.radioLayout.addWidget(self.rgbButton)
        self.radioLayout.addWidget(self.hsvButton)
        self.radioLayout.addWidget(self.yuvButton)
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
        self.hmaxSlider.setMaximum(600)
        self.hmaxSlider.setValue(600)
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
        self.sminSlider.setMaximum(100)
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
        self.smaxSlider.setMaximum(100)
        self.smaxSlider.setValue(100)
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

        ''' Vertical Layout for VMAX Slider'''
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
    
        '''Adding all the vertical layouts to the main horizontal layout'''
        self.hLayout.addLayout(self.radioLayout)
        self.hLayout.addLayout(self.hminLayout)
        self.hLayout.addLayout(self.hmaxLayout)
        self.hLayout.addLayout(self.sminLayout)
        self.hLayout.addLayout(self.smaxLayout)
        self.hLayout.addLayout(self.vminLayout)
        self.hLayout.addLayout(self.vmaxLayout)
        self.setLayout(self.hLayout)

        '''Signals for sliders value changes'''
        self.hminSlider.valueChanged.connect(self.changeHmin)     
        self.hmaxSlider.valueChanged.connect(self.changeHmax)
        self.sminSlider.valueChanged.connect(self.changeSmin)     
        self.smaxSlider.valueChanged.connect(self.changeSmax)
        self.vminSlider.valueChanged.connect(self.changeVmin)     
        self.vmaxSlider.valueChanged.connect(self.changeVmax) 

        

    #Show filtered image. Don't manually disable radio button, API does it for you!
    '''Methods for showing images depending on the current checked radio button'''
    def origButtonState(self):
        if self.origButton.isChecked():
            self.winParent.setFilter('Orig')
            

    def rgbButtonState(self):
        if self.rgbButton.isChecked():
            self.winParent.setFilter('RGB')
            

    def hsvButtonState(self):
        if self.hsvButton.isChecked():
            self.winParent.setFilter('HSV')
            

    def yuvButtonState(self):
        if self.yuvButton.isChecked():  
            self.winParent.setFilter('YUV')

    '''Methods to get the slider value and update value labels'''
    def changeHmin(self):
        value = self.hminSlider.value() / 100.0
        self.hminValue.setText(str(value))
        self.hsvLimits.hmin = value
        self.winParent.getCamera().setHSVLimits(self.hsvLimits)

    def changeHmax(self):
        value = self.hmaxSlider.value() / 100.0
        self.hmaxValue.setText(str(value))
        self.hsvLimits.hmax = value
        self.winParent.getCamera().setHSVLimits(self.hsvLimits)

    def changeSmin(self):
        value = self.sminSlider.value() / 100.0
        self.sminValue.setText(str(value))
        self.hsvLimits.smin = value
        self.winParent.getCamera().setHSVLimits(self.hsvLimits)

    def changeSmax(self):
        value = self.smaxSlider.value() / 100.0
        self.smaxValue.setText(str(value))
        self.hsvLimits.smax = value
        self.winParent.getCamera().setHSVLimits(self.hsvLimits)

    def changeVmin(self):
        value = self.vminSlider.value()
        self.vminValue.setText(str(value))
        self.hsvLimits.vmin = value
        self.winParent.getCamera().setHSVLimits(self.hsvLimits)

    def changeVmax(self):
        value = self.vmaxSlider.value()
        self.vmaxValue.setText(str(value))
        self.hsvLimits.vmax = value
        self.winParent.getCamera().setHSVLimits(self.hsvLimits)
  

    '''Close event, for finalize the program'''
    def closeEvent(self, event):
        self.winParent.closeimagesWidget()