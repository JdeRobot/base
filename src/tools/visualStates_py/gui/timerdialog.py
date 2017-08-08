import sys
from PyQt5.QtWidgets import QDialog, QGroupBox, \
    QLineEdit, QVBoxLayout, QHBoxLayout, QPushButton, \
    QWidget, QApplication
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase


class TimerDialog(QDialog):
    timeChanged = pyqtSignal('int')

    def __init__(self, name, timeValue):
        super().__init__()
        self.resize(300, 150)

        timeContainer = QGroupBox()
        timeContainer.setTitle('Time (ms)')

        self.lineEdit = QLineEdit()
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.lineEdit.setFont(fixedWidthFont)
        self.lineEdit.setText(str(timeValue))
        vLayout = QVBoxLayout()
        vLayout.addWidget(self.lineEdit)

        self.cancelButton = QPushButton()
        self.cancelButton.setText('Cancel')
        self.cancelButton.clicked.connect(self.cancelClicked)

        self.acceptButton = QPushButton()
        self.acceptButton.setText('Accept')
        self.acceptButton.clicked.connect(self.acceptClicked)

        buttonContainer = QWidget()
        hLayout = QHBoxLayout()
        hLayout.addWidget(self.cancelButton)
        hLayout.addWidget(self.acceptButton)
        buttonContainer.setLayout(hLayout)
        vLayout.addWidget(buttonContainer)
        timeContainer.setLayout(vLayout)

        vLayout2 = QVBoxLayout()
        vLayout2.addWidget(timeContainer)
        self.setLayout(vLayout2)

    def cancelClicked(self):
        self.close()

    def acceptClicked(self):
        #todo: make sure that provided value is integer
        self.timeChanged.emit(int(self.lineEdit.text()))
        self.close()
        pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = TimerDialog('Timer', 100)
    dialog.exec_()