import sys
from PyQt5.QtWidgets import QDialog, QGroupBox, \
    QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication, QLabel, QGridLayout
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase


class LibrariesDialog(QDialog):
    librariesChanged = pyqtSignal(list)

    def __init__(self, name, libraries):
        super().__init__()
        self.setWindowTitle(name)
        self.resize(300, 100)
        self.libraries = libraries
        self.libraryNameEdit = None
        self.addButton = None

        self.drawWindow()

        # self.cancelButton = QPushButton()
        # self.cancelButton.setText('Cancel')
        # self.cancelButton.clicked.connect(self.cancelClicked)
        #
        # self.acceptButton = QPushButton()
        # self.acceptButton.setText('Accept')
        # self.acceptButton.clicked.connect(self.acceptClicked)
        #
        # buttonContainer = QWidget()
        # hLayout = QHBoxLayout()
        # hLayout.addWidget(self.cancelButton)
        # hLayout.addWidget(self.acceptButton)
        # buttonContainer.setLayout(hLayout)
        # vLayout.addWidget(buttonContainer)
        # timeContainer.setLayout(vLayout)
        #
        # vLayout2 = QVBoxLayout()
        # vLayout2.addWidget(timeContainer)
        # self.setLayout(vLayout2)

    # def cancelClicked(self):
    #     self.close()
    #
    # def acceptClicked(self):
    #     #todo: make sure that provided value is integer
    #     self.timeChanged.emit(int(self.lineEdit.text()))
    #     self.close()
    #     pass

    def drawWindow(self):
        if self.layout() is not None:
            tempWidget = QWidget()
            tempWidget.setLayout(self.layout())

        gridLayout = QGridLayout()

        # add header
        gridLayout.addWidget(QLabel('Libraries'), 0, 0)
        gridLayout.addWidget(QLabel(''), 0, 1)

        # add new library edit box
        self.libraryNameEdit = QLineEdit()
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.libraryNameEdit.setFont(fixedWidthFont)
        gridLayout.addWidget(self.libraryNameEdit, 1, 0)
        self.addButton = QPushButton('Add')
        self.addButton.clicked.connect(self.addClicked)
        gridLayout.addWidget(self.addButton, 1, 1)

        self.buttons = {}

        row = 2
        for lib in self.libraries:
            gridLayout.addWidget(QLabel(lib), row, 0)
            deleteButton = QPushButton()
            deleteButton.setObjectName(lib)
            deleteButton.setText('Delete')
            deleteButton.clicked.connect(self.deleteButtonClicked)
            gridLayout.addWidget(deleteButton, row, 1)
            row += 1
            self.buttons[deleteButton] = lib

        self.resize(300, 100)
        self.setLayout(gridLayout)


    def deleteButtonClicked(self):
        self.libraries.remove(self.sender().objectName())
        self.drawWindow()
        self.librariesChanged.emit(self.libraries)

    def addClicked(self):
        self.libraries.append(self.libraryNameEdit.text())
        self.drawWindow()
        self.librariesChanged.emit(self.libraries)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = LibrariesDialog('Libraries', ['okan'])
    dialog.exec_()





