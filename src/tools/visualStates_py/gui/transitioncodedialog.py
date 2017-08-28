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
from PyQt5.QtWidgets import QDialog, QTextEdit, QPushButton, QVBoxLayout, QWidget, QHBoxLayout, QApplication, QRadioButton, QGroupBox
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFontDatabase
from gui.transitiontype import TransitionType

class TransitionCodeDialog(QDialog):
    codeChanged = pyqtSignal('int', 'QString', 'QString')

    def __init__(self, name, transition):
        super(QDialog, self).__init__()
        self.transition = transition
        self.setWindowTitle(name)
        self.resize(800, 600)
        self.codeEdit = QTextEdit()
        self.codeEdit.setText(self.transition.getCode())
        fixedWidthFont = QFontDatabase.systemFont(QFontDatabase.FixedFont)
        self.codeEdit.setFont(fixedWidthFont)
        self.cancelButton = QPushButton('Cancel')
        self.cancelButton.clicked.connect(self.cancel)
        self.acceptButton = QPushButton('Accept')
        self.acceptButton.clicked.connect(self.accept)

        self.temporalButton = QRadioButton('Temporal', self)
        self.temporalButton.toggled.connect(self.temporalToggled)
        self.conditionalButton = QRadioButton('Conditional', self)
        self.conditionalButton.toggled.connect(self.conditionalToggled)

        radioButtonContainer = QGroupBox()
        radioButtonContainer.setTitle('Transition Type')
        vLayout = QVBoxLayout()
        vLayout.addWidget(self.temporalButton)
        vLayout.addWidget(self.conditionalButton)
        radioButtonContainer.setLayout(vLayout)

        self.transitionTypeCode = QTextEdit()
        self.transitionTypeCode.setFont(fixedWidthFont)
        self.transitionGroupBox = QGroupBox()
        self.transitionGroupBox.setTitle('Temporal (number in ms)')
        h3Layout = QHBoxLayout()
        h3Layout.addWidget(self.transitionTypeCode)
        self.transitionGroupBox.setLayout(h3Layout)

        typeContainer = QWidget()
        h2Layout = QHBoxLayout()
        h2Layout.addWidget(radioButtonContainer)
        h2Layout.addWidget(self.transitionGroupBox)
        typeContainer.setLayout(h2Layout)

        verticalLayout = QVBoxLayout()
        verticalLayout.addWidget(typeContainer)
        verticalLayout.addWidget(self.codeEdit)

        container = QWidget()
        hLayout =QHBoxLayout()
        hLayout.addWidget(self.cancelButton)
        hLayout.addWidget(self.acceptButton)
        container.setLayout(hLayout)

        verticalLayout.addWidget(container)
        self.setLayout(verticalLayout)

        if self.transition.getType() == TransitionType.CONDITIONAL:
            self.conditionalButton.setChecked(True)
            self.conditionalToggled()
        elif self.transition.getType() == TransitionType.TEMPORAL:
            self.temporalButton.setChecked(True)
            self.temporalToggled()

    def cancel(self):
        self.close()

    def accept(self):
        type = None
        typeValue = None

        if self.temporalButton.isChecked():
            type = int(TransitionType.TEMPORAL)
            typeValue = self.transitionTypeCode.toPlainText()
        elif self.conditionalButton.isChecked():
            type = int(TransitionType.CONDITIONAL)
            typeValue = self.transitionTypeCode.toPlainText()

        self.codeChanged.emit(type, typeValue, self.codeEdit.toPlainText())
        self.close()

    def temporalToggled(self):
        if (self.temporalButton.isChecked()):
            self.transitionGroupBox.setTitle('Temporal (number in ms)')
            self.transitionTypeCode.setPlainText(str(self.transition.getTemporalTime()))
            print('temporal toggled')

    def conditionalToggled(self):
        if (self.conditionalButton.isChecked()):
            self.transitionGroupBox.setTitle('Condition (evaluates to true or false)')
            self.transitionTypeCode.setPlainText(self.transition.getCondition())
            print('conditional toggled')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    dialog = TransitionCodeDialog('Rename', 'Hello World')
    dialog.exec_()
