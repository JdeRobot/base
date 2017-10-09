#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
sys.path.append("/opt/jderobot/lib/python3.5/visualStates_py")

from PyQt5.QtWidgets import QApplication
from codegen.python.runtimegui import RunTimeGui

gui = None

def runGui():
	global gui
	app = QApplication(sys.argv)
	gui = RunTimeGui()
	gui.activateIPC()

	gui.addState(0, "root", True, 0.0, 0.0, None)
	gui.addState(1, "moving", True, 781.0, 764.0, 0)
	gui.addState(2, "stop", False, 944.0, 938.0, 0)
	gui.addState(3, "state 3", True, 817.0, 800.0, 1)
	gui.addState(4, "state 4", False, 984.0, 979.0, 1)

	gui.addTransition(1, "10secs", 1, 2, 803.5, 929.0)
	gui.addTransition(5, "transition 5", 2, 1, 903.5, 800.0)
	gui.addTransition(2, "transition 2", 3, 4, 865.5, 977.5)
	gui.addTransition(4, "transition 4", 4, 3, 969.5, 832.5)

	gui.emitLoadFromRoot()
	gui.emitActiveStateById(0)
	gui.show()
	app.exec_()

if __name__ == "__main__":
	runGui()

