'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
import sys
from gui.interfaces import Interfaces
from PyQt5.QtWidgets import QDialog, QGroupBox, \
    QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication, QLabel, QGridLayout, QComboBox
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase

class ConfigDialog(QDialog):
    configChanged = pyqtSignal(list)

    def __init__(self, name, configs):
        super(QDialog, self).__init__()
        self.setWindowTitle(name)
        self.configs = configs
        self.serverTypeCombo = None
        self.nameEdit = None
        self.topicEdit = None
        self.proxyNameEdit = None
        self.ipEdit = None
        self.portEdit = None
        self.interfaceCombo = None
        self.addButton = None

        self.drawWindow()

    def drawWindow(self):
        if self.layout() is not None:
            tempWidget = QWidget()
            tempWidget.setLayout(self.layout())

        gridLayout = QGridLayout()

        # add header
        gridLayout.addWidget(QLabel('Server Type'), 0, 0)
        gridLayout.addWidget(QLabel('Name'), 0, 1)
        gridLayout.addWidget(QLabel('Topic'), 0, 2)
        gridLayout.addWidget(QLabel('Proxy Name'), 0, 3)
        gridLayout.addWidget(QLabel('IP'), 0, 4)
        gridLayout.addWidget(QLabel('Port'), 0, 5)
        gridLayout.addWidget(QLabel('Interface'), 0, 6)
        gridLayout.addWidget(QLabel(''), 0, 7)

        # add new config input fields
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.serverTypeCombo = QComboBox()
        self.serverTypeCombo.setFont(fixedWidthFont)
        gridLayout.addWidget(self.serverTypeCombo, 1, 0)
        self.nameEdit = QLineEdit()
        self.nameEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.nameEdit, 1, 1)
        self.topicEdit = QLineEdit()
        self.topicEdit.setFont(fixedWidthFont)
        self.topicEdit.setEnabled(False)
        gridLayout.addWidget(self.topicEdit, 1, 2)
        self.proxyNameEdit = QLineEdit()
        self.proxyNameEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.proxyNameEdit, 1, 3)
        self.ipEdit = QLineEdit()
        self.ipEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.ipEdit, 1, 4)
        self.portEdit = QLineEdit()
        self.portEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.portEdit, 1, 5)
        self.interfaceCombo = QComboBox()
        gridLayout.addWidget(self.interfaceCombo, 1, 6)
        self.addButton = QPushButton('Add')
        gridLayout.addWidget(self.addButton, 1, 7)
        self.addButton.clicked.connect(self.addClicked)

        # add server types to the combobox
        self.serverTypeCombo.addItem('ICE', 'ice')
        self.serverTypeCombo.addItem('ROS', 'ros')
        self.serverTypeCombo.currentIndexChanged.connect(self.serverTypeChanged)

        # add interfaces to the combobox
        interfaces = Interfaces.getInterfaces()
        for interfaceName in interfaces:
            self.interfaceCombo.addItem(interfaceName, interfaceName)

        row = 2
        for configItem in self.configs:
            gridLayout.addWidget(QLabel(configItem['serverType']), row, 0)
            gridLayout.addWidget(QLabel(configItem['name']), row, 1)
            gridLayout.addWidget(QLabel(configItem['topic']), row, 2)
            gridLayout.addWidget(QLabel(configItem['proxyName']), row, 3)
            gridLayout.addWidget(QLabel(configItem['ip']), row, 4)
            gridLayout.addWidget(QLabel(configItem['port']), row, 5)
            gridLayout.addWidget(QLabel(configItem['interface']), row, 6)
            deleteButton = QPushButton('Delete')
            deleteButton.clicked.connect(self.deleteClicked)
            # we will find the item to be deleted based on the index on the config list
            deleteButton.setObjectName(str(row-2))
            gridLayout.addWidget(deleteButton, row, 7)
            row += 1

        self.resize(700, 100)
        self.setLayout(gridLayout)

    def serverTypeChanged(self):
        if self.serverTypeCombo.currentData() == 'ros':
            self.topicEdit.setEnabled(True)
            self.proxyNameEdit.setEnabled(False)
            self.ipEdit.setEnabled(False)
            self.portEdit.setEnabled(False)
        elif self.serverTypeCombo.currentData() == 'ice':
            self.topicEdit.setEnabled(False)
            self.proxyNameEdit.setEnabled(True)
            self.ipEdit.setEnabled(True)
            self.portEdit.setEnabled(True)

    def deleteClicked(self):
        dataIndex = int(self.sender().objectName())
        print('dataIndex:' + str(dataIndex))
        self.configs.pop(dataIndex)
        self.configChanged.emit(self.configs)
        self.drawWindow()

    def addClicked(self):
        configData = {}
        configData['serverType'] = self.serverTypeCombo.currentData()
        configData['name'] = self.nameEdit.text()
        configData['topic'] = self.topicEdit.text()
        configData['proxyName'] = self.proxyNameEdit.text()
        configData['ip'] = self.ipEdit.text()
        configData['port'] = self.portEdit.text()
        configData['interface'] = self.interfaceCombo.currentData()
        self.configs.append(configData)
        self.configChanged.emit(self.configs)
        self.drawWindow()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = ConfigDialog('Config', [])
    dialog.exec_()