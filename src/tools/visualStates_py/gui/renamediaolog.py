import sys
from PyQt5.QtWidgets import QDialog, QLineEdit, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QApplication
from PyQt5.QtCore import pyqtSignal

class RenameDialog(QDialog):
    nameChanged = pyqtSignal('QString')

    def __init__(self, name, currentValue):
        super().__init__()
        self.setWindowTitle(name)
        self.nameEdit = QLineEdit()
        self.nameEdit.setText(currentValue)
        self.cancelButton = QPushButton('Cancel')
        self.cancelButton.clicked.connect(self.cancel)
        self.acceptButton = QPushButton('Accept')
        self.acceptButton.clicked.connect(self.accept)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.nameEdit)

        container = QWidget()
        hLayout =QHBoxLayout()
        hLayout.addWidget(self.cancelButton)
        hLayout.addWidget(self.acceptButton)
        container.setLayout(hLayout)

        verticalLayout.addWidget(container)
        self.setLayout(verticalLayout)

    def cancel(self):
        self.close()

    def accept(self):
        self.nameChanged.emit(self.nameEdit.text())
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = RenameDialog('Rename', 'Hello World')
    dialog.exec_()
