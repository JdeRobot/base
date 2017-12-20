#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
sys.path.append("/opt/jderobot/lib/python2.7/visualStates_py")

from PyQt5.QtWidgets import QApplication
from codegen.python.runtimegui import RunTimeGui

gui = None

def runGui():
	global gui
	app = QApplication(sys.argv)
	gui = RunTimeGui()
	gui.activateIPC()

	gui.addState(0, "root", True, 0.0, 0.0, None)
	gui.addState(1, "state 1", True, 903.0, 922.0, 0)
	gui.addState(2, "state 2", False, 1031.0, 1096.0, 0)

	gui.addTransition(1, "transition 1", 1, 2, 1025.0, 954.0)
	gui.addTransition(2, "transition 2", 2, 1, 860.0, 1054.0)

	gui.emitLoadFromRoot()
	gui.emitActiveStateById(0)
	gui.show()
	app.exec_()

if __name__ == "__main__":
	runGui()

