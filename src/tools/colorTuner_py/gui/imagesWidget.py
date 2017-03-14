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

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QLabel
from PyQt5.QtGui import QImage, QPixmap

class ImagesWidget(QWidget):
    IMAGE_COLS_MAX=640
    IMAGE_ROWS_MAX=360
    
    imageUpdate=pyqtSignal()
    
    def __init__(self,winParent):      
        super(ImagesWidget, self).__init__()
        self.winParent=winParent
        self.imageUpdate.connect(self.updateImage)
        self.initUI()
        
    def initUI(self):

        self.setMinimumSize(1340,400)
        self.setMaximumSize(1340,400)

        self.imgLabelOrig=QLabel(self)
        self.imgLabelOrig.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgLabelOrig.move(20,20)
        self.imgLabelOrig.show()

        self.imgLabelFiltered=QLabel(self)
        self.imgLabelFiltered.resize(self.IMAGE_COLS_MAX,self.IMAGE_ROWS_MAX)
        self.imgLabelFiltered.move(40 + self.IMAGE_COLS_MAX,20)
        self.imgLabelFiltered.show()

    def setOrigImage(self):
        img = self.winParent.getCamera().getOrigImage()

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelOrig.resize(img.shape[1],img.shape[0])
            self.imgLabelOrig.setPixmap(QPixmap.fromImage(image))

    def setFilteredImage(self):
        filt = self.winParent.getFilterName()
        img = self.winParent.getCamera().getFilteredImage(filt)

        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888)
            self.imgLabelFiltered.resize(img.shape[1],img.shape[0])
            self.imgLabelFiltered.setPixmap(QPixmap.fromImage(image))
        
    def updateImage(self):
        self.setOrigImage()
        self.setFilteredImage()
        
    def closeEvent(self, event):
        self.winParent.closeimagesWidget()