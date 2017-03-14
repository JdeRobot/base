#
#  Copyright (C) 1997-2016 JDE Developers Team
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
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QImage, QPixmap


class CameraWidget(QtWidgets.QWidget):

    imageUpdate = QtCore.pyqtSignal()

    def __init__(self, winParent):
        super(CameraWidget, self).__init__()
        self.winParent = winParent
        self.imageUpdate.connect(self.updateImage)
        self.initUI()

    def initUI(self):

        self.setWindowTitle("Camera")
        self.setMinimumSize(100,100)

        self.imgLabel = QtWidgets.QLabel(self)
        self.imgLabel.show()

    def updateImage(self):

        if (self.winParent.getCamera()):
            img = self.winParent.getCamera().getImage().data
            if img is not None:
                image = QImage(img.data, img.shape[1], img.shape[0],
                                     img.shape[1] * img.shape[2], QImage.Format_RGB888)

                size = QtCore.QSize(img.shape[1], img.shape[0])
                self.resize(size)
                self.imgLabel.resize(size)
                self.imgLabel.setPixmap(QPixmap.fromImage(image))
