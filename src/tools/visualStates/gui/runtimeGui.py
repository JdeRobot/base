# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'runtimeGui.ui'
#
# Created: Sun Jan 31 12:00:35 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_visualStates(object):
    def setupUi(self, visualStates):
        visualStates.setObjectName(_fromUtf8("visualStates"))
        visualStates.resize(761, 553)
        self.centralwidget = QtGui.QWidget(visualStates)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.frameTreeView = QtGui.QFrame(self.centralwidget)
        self.frameTreeView.setMaximumSize(QtCore.QSize(250, 16777215))
        self.frameTreeView.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frameTreeView.setFrameShadow(QtGui.QFrame.Raised)
        self.frameTreeView.setLineWidth(1)
        self.frameTreeView.setObjectName(_fromUtf8("frameTreeView"))
        self.verticalLayout_10 = QtGui.QVBoxLayout(self.frameTreeView)
        self.verticalLayout_10.setSpacing(3)
        self.verticalLayout_10.setMargin(0)
        self.verticalLayout_10.setObjectName(_fromUtf8("verticalLayout_10"))
        self.TreeViewLabel = QtGui.QLabel(self.frameTreeView)
        self.TreeViewLabel.setObjectName(_fromUtf8("TreeViewLabel"))
        self.verticalLayout_10.addWidget(self.TreeViewLabel)
        self.treeView = QtGui.QTreeView(self.frameTreeView)
        self.treeView.setObjectName(_fromUtf8("treeView"))
        self.verticalLayout_10.addWidget(self.treeView)
        self.autofocus = QtGui.QCheckBox(self.frameTreeView)
        self.autofocus.setObjectName(_fromUtf8("autofocus"))
        self.verticalLayout_10.addWidget(self.autofocus)
        self.upButton = QtGui.QPushButton(self.frameTreeView)
        self.upButton.setObjectName(_fromUtf8("upButton"))
        self.verticalLayout_10.addWidget(self.upButton)
        self.horizontalLayout.addWidget(self.frameTreeView)
        self.frameSchema = QtGui.QFrame(self.centralwidget)
        self.frameSchema.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frameSchema.setFrameShadow(QtGui.QFrame.Raised)
        self.frameSchema.setObjectName(_fromUtf8("frameSchema"))
        self.verticalLayout_9 = QtGui.QVBoxLayout(self.frameSchema)
        self.verticalLayout_9.setSpacing(3)
        self.verticalLayout_9.setMargin(0)
        self.verticalLayout_9.setObjectName(_fromUtf8("verticalLayout_9"))
        self.SchemaLabel = QtGui.QLabel(self.frameSchema)
        self.SchemaLabel.setObjectName(_fromUtf8("SchemaLabel"))
        self.verticalLayout_9.addWidget(self.SchemaLabel)
        self.schemaView = QtGui.QGraphicsView(self.frameSchema)
        self.schemaView.setObjectName(_fromUtf8("schemaView"))
        self.verticalLayout_9.addWidget(self.schemaView)
        self.horizontalLayout.addWidget(self.frameSchema)
        visualStates.setCentralWidget(self.centralwidget)

        self.retranslateUi(visualStates)
        QtCore.QMetaObject.connectSlotsByName(visualStates)

    def retranslateUi(self, visualStates):
        visualStates.setWindowTitle(_translate("visualStates", "visualStates - runtimeGUI", None))
        self.TreeViewLabel.setText(_translate("visualStates", "Tree VIew", None))
        self.autofocus.setText(_translate("visualStates", "Autofocus", None))
        self.upButton.setText(_translate("visualStates", "Up", None))
        self.SchemaLabel.setText(_translate("visualStates", "Schema", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    visualStates = QtGui.QMainWindow()
    ui = Ui_visualStates()
    ui.setupUi(visualStates)
    visualStates.show()
    sys.exit(app.exec_())

