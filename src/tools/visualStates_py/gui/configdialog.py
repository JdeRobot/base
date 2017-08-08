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
        super().__init__()
        self.setWindowTitle(name)
        self.configs = configs
        self.nameEdit = None
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
        gridLayout.addWidget(QLabel('Name'), 0, 0)
        gridLayout.addWidget(QLabel('Proxy Name'), 0, 1)
        gridLayout.addWidget(QLabel('IP'), 0, 2)
        gridLayout.addWidget(QLabel('Port'), 0, 3)
        gridLayout.addWidget(QLabel('Interface'), 0, 4)
        gridLayout.addWidget(QLabel(''), 0, 5)


        # add new config input fields
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.nameEdit = QLineEdit()
        self.nameEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.nameEdit, 1, 0)
        self.proxyNameEdit = QLineEdit()
        self.proxyNameEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.proxyNameEdit, 1, 1)
        self.ipEdit = QLineEdit()
        self.ipEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.ipEdit, 1, 2)
        self.portEdit = QLineEdit()
        self.portEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.portEdit, 1, 3)
        self.interfaceCombo = QComboBox()
        gridLayout.addWidget(self.interfaceCombo, 1, 4)
        self.addButton = QPushButton('Add')
        gridLayout.addWidget(self.addButton, 1, 5)
        self.addButton.clicked.connect(self.addClicked)

        # add interfaces to the combobox
        interfaces = Interfaces.getInterfaces()
        for interfaceName in interfaces:
            self.interfaceCombo.addItem(interfaceName, interfaces[interfaceName])

        row = 2
        for configItem in self.configs:
            gridLayout.addWidget(QLabel(configItem['name']), row, 0)
            gridLayout.addWidget(QLabel(configItem['proxyName']), row, 1)
            gridLayout.addWidget(QLabel(configItem['ip']), row, 2)
            gridLayout.addWidget(QLabel(configItem['port']), row, 3)
            gridLayout.addWidget(QLabel(configItem['interface']), row, 4)
            deleteButton = QPushButton('Delete')
            deleteButton.clicked.connect(self.deleteClicked)
            # we will find the item to be deleted based on the index on the config list
            deleteButton.setObjectName(str(row-2))
            gridLayout.addWidget(deleteButton, row, 5)
            row += 1

        self.resize(700, 100)
        self.setLayout(gridLayout)


    def deleteClicked(self):
        dataIndex = int(self.sender().objectName())
        print('dataIndex:' + str(dataIndex))
        self.configs.pop(dataIndex)
        self.configChanged.emit(self.configs)
        self.drawWindow()

    def addClicked(self):
        configData = {}
        configData['name'] = self.nameEdit.text()
        configData['proxyName'] = self.proxyNameEdit.text()
        configData['ip'] = self.ipEdit.text()
        configData['port'] = self.portEdit.text()
        interface = self.interfaceCombo.currentData()
        configData['interface'] = interface
        self.configs.append(configData)
        self.configChanged.emit(self.configs)
        self.drawWindow()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = ConfigDialog('Config', [])
    dialog.exec_()