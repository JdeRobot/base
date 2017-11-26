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
	gui.addState(1, "state 1", True, 848.0, 820.0, 0)
	gui.addState(2, "state 2", False, 1050.0, 1061.0, 0)

	gui.addTransition(1, "transition 1", 1, 2, 1029.0, 867.5)
	gui.addTransition(2, "transition 2", 2, 1, 853.0, 968.5)

	gui.emitLoadFromRoot()
	gui.emitActiveStateById(0)
	gui.show()
	app.exec_()

if __name__ == "__main__":
	runGui()

