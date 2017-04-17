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

from PyQt4.QtCore import pyqtSignal, Qt
from PyQt4.QtGui import QWidget, QLabel, QRadioButton, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy, QSlider
from filters.hsvFilter import HSVMAX, HSVMIN
from filters.rgbFilter import RGBMAX, RGBMIN
from filters.yuvFilter import YUVMAX, YUVMIN
from .logoWidget import  LogoWidget

WIDTH = 1340
HEIGTH = 200

class ControlWidget(QWidget):
    
    
    def __init__(self,winParent):      
        super(ControlWidget, self).__init__()
        self.winParent=winParent

        self.rgbdwn = RGBMIN
        self.rgbup = RGBMAX
        self.hsvdwn = HSVMIN
        self.hsvup = HSVMAX
        self.yuvdwn = YUVMIN
        self.yuvup = YUVMAX
        self.initUI()
        
    def initUI(self):

        self.setMinimumSize(WIDTH,HEIGTH)
        self.setMaximumSize(WIDTH,HEIGTH)

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


        self.logoLayout = QVBoxLayout()
        self.logo = LogoWidget(self, HEIGTH, HEIGTH)
        self.logoLayout.addWidget(self.logo)

        

        
        hmin,smin,vmin = HSVMIN
        hmax,smax,vmax = HSVMAX
        ''' Vertical Layout for HMIN Slider''' 
        self.hminLayout = QVBoxLayout()
        self.hminLayout.setObjectName("hminLayout")
        self.hminLabel = QLabel("HMin")
        self.hminValue = QLabel("0")
        self.hminValue.setAlignment(Qt.AlignCenter);
        self.hminSlider = QSlider(Qt.Vertical)
        self.hminSlider.setMinimum(hmin)
        self.hminSlider.setMaximum(hmax)
        self.hminSlider.setValue(hmin)
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
        self.hmaxSlider.setMinimum(hmin)
        self.hmaxSlider.setMaximum(hmax)
        self.hmaxSlider.setValue(hmax)
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
        self.sminSlider.setMinimum(smin)
        self.sminSlider.setMaximum(smax)
        self.sminSlider.setValue(smin)
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
        self.smaxSlider.setMinimum(smin)
        self.smaxSlider.setMaximum(smax)
        self.smaxSlider.setValue(smax)
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
        self.vminSlider.setMinimum(vmin)
        self.vminSlider.setMaximum(vmax)
        self.vminSlider.setValue(vmin)
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
        self.vmaxSlider.setMinimum(vmin)
        self.vmaxSlider.setMaximum(vmax)
        self.vmaxSlider.setValue(vmax)
        self.vmaxLayout.addWidget(self.vmaxLabel)
        self.vmaxLayout.addWidget(self.vmaxValue)
        self.vmaxLayout.addWidget(self.vmaxSlider)
    
        '''Adding all the vertical layouts to the main horizontal layout'''
        self.hLayout.addLayout(self.radioLayout)
        self.hLayout.addLayout(self.logoLayout)
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
            self.winParent.setFilterName('Orig')
            

    def rgbButtonState(self):
        if self.rgbButton.isChecked():
            self.winParent.setFilterName('RGB')

            rmin,gmin,bmin = RGBMIN
            rmax,gmax,bmax = RGBMAX
            print(RGBMAX)
            rd, gd, bd = self.rgbdwn
            ru, gu, bu = self.rgbup
            self.hminLabel.setText('Rmin')
            self.hminValue.setText(str(rd))
            self.hminSlider.setMinimum(rmin)
            self.hminSlider.setMaximum(rmax)
            self.hminSlider.setValue(rd)

            self.hmaxLabel.setText("RMax")
            self.hmaxValue.setText(str(ru))
            self.hmaxSlider.setMinimum(rmin)
            self.hmaxSlider.setMaximum(rmax)
            self.hmaxSlider.setValue(ru)

            self.sminLabel.setText("GMin")
            self.sminValue.setText(str(gd))
            self.sminSlider.setMinimum(gmin)
            self.sminSlider.setMaximum(gmax)
            self.sminSlider.setValue(gd)

            self.smaxLabel.setText("GMax")
            self.smaxValue.setText(str(gu))
            self.smaxSlider.setMinimum(gmin)
            self.smaxSlider.setMaximum(gmax)
            self.smaxSlider.setValue(gu)

            self.vminLabel.setText("BMin")
            self.vminValue.setText(str(bd))
            self.vminSlider.setMinimum(bmin)
            self.vminSlider.setMaximum(bmax)
            self.vminSlider.setValue(bd)

            self.vmaxLabel.setText("BMax")
            self.vmaxValue.setText(str(bu))
            self.vmaxSlider.setMinimum(bmin)
            self.vmaxSlider.setMaximum(bmax)
            self.vmaxSlider.setValue(bu)

            

    def hsvButtonState(self):
        if self.hsvButton.isChecked():
            self.winParent.setFilterName('HSV')

            hmin,smin,vmin = HSVMIN
            hmax,smax,vmax = HSVMAX

            hd, sd, vd = self.hsvdwn
            hu, su, vu = self.hsvup
            
            self.hminLabel.setText("HMin")
            self.hminValue.setText(str(hd))
            self.hminSlider.setMinimum(hmin)
            self.hminSlider.setMaximum(hmax)
            self.hminSlider.setValue(hd)

            self.hmaxLabel.setText("HMax")
            self.hmaxValue.setText(str(hu))
            self.hmaxSlider.setMinimum(hmin)
            self.hmaxSlider.setMaximum(hmax)
            self.hmaxSlider.setValue(hu)

            self.sminLabel.setText("SMin")
            self.sminValue.setText(str(sd))
            self.sminSlider.setMinimum(smin)
            self.sminSlider.setMaximum(smax)
            self.sminSlider.setValue(sd)

            self.smaxLabel.setText("SMax")
            self.smaxValue.setText(str(su))
            self.smaxSlider.setMinimum(smin)
            self.smaxSlider.setMaximum(smax)
            self.smaxSlider.setValue(su)

            self.vminLabel.setText("VMin")
            self.vminValue.setText(str(vd))
            self.vminSlider.setMinimum(vmin)
            self.vminSlider.setMaximum(vmax)
            self.vminSlider.setValue(vd)

            self.vmaxLabel.setText("VMax")
            self.vmaxValue.setText(str(vu))
            self.vmaxSlider.setMinimum(vmin)
            self.vmaxSlider.setMaximum(vmax)
            self.vmaxSlider.setValue(vu)
            

    def yuvButtonState(self):
        if self.yuvButton.isChecked():  
            self.winParent.setFilterName('YUV')

            ymin,umin,vmin = YUVMIN
            ymax,umax,vmax = YUVMAX

            yd, ud, vd = self.yuvdwn
            yu, uu, vu = self.yuvup
            
            self.hminLabel.setText("YMin")
            self.hminValue.setText(str(yd))
            self.hminSlider.setMinimum(ymin)
            self.hminSlider.setMaximum(ymax)
            self.hminSlider.setValue(yd)

            self.hmaxLabel.setText("YMax")
            self.hmaxValue.setText(str(yu))
            self.hmaxSlider.setMinimum(ymin)
            self.hmaxSlider.setMaximum(ymax)
            self.hmaxSlider.setValue(yu)

            self.sminLabel.setText("UMin")
            self.sminValue.setText(str(ud))
            self.sminSlider.setMinimum(umin)
            self.sminSlider.setMaximum(umax)
            self.sminSlider.setValue(ud)

            self.smaxLabel.setText("UMax")
            self.smaxValue.setText(str(uu))
            self.smaxSlider.setMinimum(umin)
            self.smaxSlider.setMaximum(umax)
            self.smaxSlider.setValue(uu)

            self.vminLabel.setText("VMin")
            self.vminValue.setText(str(vd))
            self.vminSlider.setMinimum(vmin)
            self.vminSlider.setMaximum(vmax)
            self.vminSlider.setValue(vd)

            self.vmaxLabel.setText("VMax")
            self.vmaxValue.setText(str(vu))
            self.vmaxSlider.setMinimum(vmin)
            self.vmaxSlider.setMaximum(vmax)
            self.vmaxSlider.setValue(vu)

    '''Methods to get the slider value and update value labels'''
    def changeHmin(self):
        value = self.hminSlider.value()
        if self.hsvButton.isChecked():
            self.hsvdwn[0] = value
        elif self.rgbButton.isChecked():
            self.rgbdwn[0] = value
        elif self.yuvButton.isChecked():
            value = self.hminSlider.value()
            self.yuvdwn[0] = value
        self.hminValue.setText(str(value))
        self.setMIN()
       

    def changeHmax(self):
        value = self.hmaxSlider.value()
        if self.hsvButton.isChecked():
            self.hsvup[0] = value
        elif self.rgbButton.isChecked():
            self.rgbup[0] = value
        elif self.yuvButton.isChecked():
            self.yuvup[0] = value

        self.hmaxValue.setText(str(value))
        self.setMAX()

    def changeSmin(self):
        value = self.sminSlider.value()
        if self.hsvButton.isChecked():
            self.hsvdwn[1] = value
        elif self.rgbButton.isChecked():
            self.rgbdwn[1] = value
        elif self.yuvButton.isChecked():
            self.yuvdwn[1] = value

        self.sminValue.setText(str(value))
        self.setMIN()

    def changeSmax(self):
        value = self.smaxSlider.value()
        if self.hsvButton.isChecked():
            self.hsvup[1] = value
        elif self.rgbButton.isChecked():
            self.rgbup[1] = value
        elif self.yuvButton.isChecked():
            self.yuvup[1] = value

        self.smaxValue.setText(str(value))
        self.setMAX()

    def changeVmin(self):
        value = self.vminSlider.value()
        if self.hsvButton.isChecked():
            self.hsvdwn[2] = value
        elif self.rgbButton.isChecked():
            self.rgbdwn[2] = value
        elif self.yuvButton.isChecked():
            self.yuvdwn[2] = value

        self.vminValue.setText(str(value))
        self.setMIN()

    def changeVmax(self):
        value = self.vmaxSlider.value()
        if self.hsvButton.isChecked():
            self.hsvup[2] = value
        elif self.rgbButton.isChecked():
            self.rgbup[2] = value
        elif self.yuvButton.isChecked():
            self.yuvup[2] = value

        self.vmaxValue.setText(str(value))
        self.setMAX()

    def setMAX (self):

        filt = self.winParent.getFilterName()
        if self.hsvButton.isChecked():
            h, s, v = self.hsvup
            self.winParent.getCamera().getFilter(filt).setUpLimit(h,s,v)
        elif self.rgbButton.isChecked():
            h, s, v = self.rgbup
            self.winParent.getCamera().getFilter(filt).setUpLimit(h,s,v)
        elif self.yuvButton.isChecked():
            h, s, v = self.yuvup
            self.winParent.getCamera().getFilter(filt).setUpLimit(h,s,v)

    def setMIN (self):
        filt = self.winParent.getFilterName()
        if self.hsvButton.isChecked():
            h, s, v = self.hsvdwn
            self.winParent.getCamera().getFilter(filt).setDownLimit(h,s,v)
        elif self.rgbButton.isChecked():
            h, s, v = self.rgbdwn
            self.winParent.getCamera().getFilter(filt).setDownLimit(h,s,v)
        elif self.yuvButton.isChecked():
            h, s, v = self.yuvdwn
            self.winParent.getCamera().getFilter(filt).setDownLimit(h,s,v)

  

    '''Close event, for finalize the program'''
    def closeEvent(self, event):
        self.winParent.closeimagesWidget()