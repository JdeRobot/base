import sys
from PyQt5.QtWidgets import QDialog, QTextEdit, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QApplication
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase

class CodeDialog(QDialog):
    codeChanged = pyqtSignal('QString')

    def __init__(self, name, currentValue):
        super().__init__()
        self.setWindowTitle(name)
        self.resize(800, 600)
        self.codeEdit = QTextEdit()
        self.codeEdit.setText(currentValue)
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.codeEdit.setFont(fixedWidthFont)
        self.cancelButton = QPushButton('Cancel')
        self.cancelButton.clicked.connect(self.cancel)
        self.acceptButton = QPushButton('Accept')
        self.acceptButton.clicked.connect(self.accept)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(self.codeEdit)

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
        self.codeChanged.emit(self.codeEdit.toPlainText())
        self.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = CodeDialog('Rename', 'Hello World')
    dialog.exec_()
