#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys, threading, time, signal
sys.path.append("/opt/jderobot/lib/python2.7")
sys.path.append("/opt/jderobot/lib/python2.7/visualStates_py")
from codegen.python.state import State
from codegen.python.temporaltransition import TemporalTransition
from codegen.python.conditionaltransition import ConditionalTransition
from codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication
import config, comm

import comm

class State0(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass


class State1(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		self.interfaces.myMotors.sendV(0.1)


class State2(State):
	def __init__(self, id, initial, interfaces, cycleDuration, parent=None, gui=None):
		State.__init__(self, id, initial, cycleDuration, parent, gui)
		self.interfaces = interfaces

	def runCode(self):
		pass


class Tran1(TemporalTransition):

	def runCode(self):
		pass

class Tran2(TemporalTransition):

	def runCode(self):
		pass

class Interfaces():
	def __init__(self):
		self.jdrc = None
		self.myMotors = None
		self.connectProxies()

	def connectProxies(self):
		cfg = config.load(sys.argv[1])
		self.jdrc = comm.init(cfg, "sample1")
		self.myMotors = self.jdrc.getMotorsClient("sample1.myMotors")
		if not self.myMotors:
			raise Exception("could not create client with name:myMotors")
		print("myMotors is connected")

	def destroyProxies(self):
		if self.jdrc is not None:
			self.jdrc.destroy()

displayGui = False
guiThread = None
gui = None
state0 = None

def signal_handler(signal, frame):
	global gui
	print("SIGINT is captured. The program exits")
	if gui is not None:
		gui.close()
	global state0
	state0.stop()

def readArgs():
	global displayGui
	for arg in sys.argv:
		splitedArg = arg.split('=')
		if splitedArg[0] == '--displaygui':
			if splitedArg[1] == 'True' or splitedArg[1] == 'true':
				displayGui = True
				print('runtime gui enabled')
			else:
				displayGui = False
				print('runtime gui disabled')

def runGui():
	global gui
	app = QApplication(sys.argv)
	gui = RunTimeGui()
	gui.show()
	app.exec_()

if __name__ == "__main__":
	interfaces = Interfaces()

	readArgs()
	if displayGui:
		guiThread = threading.Thread(target=runGui)
		guiThread.start()


	if displayGui:
		while(gui is None):
			time.sleep(0.1)

		gui.addState(0, "root", True, 0.0, 0.0, None)
		gui.addState(1, "state 1", True, 848.0, 820.0, 0)
		gui.addState(2, "state 2", False, 1050.0, 1061.0, 0)

		gui.addTransition(1, "transition 1", 1, 2, 1029.0, 867.5)
		gui.addTransition(2, "transition 2", 2, 1, 853.0, 968.5)

	if displayGui:
		gui.emitLoadFromRoot()
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state2 = State2(2, False, interfaces, 100, state0, gui)

	tran1 = Tran1(1, 2, 1000)
	state1.addTransition(tran1)

	tran2 = Tran2(2, 1, 1000)
	state2.addTransition(tran2)

	try:
		state0.startThread()
		signal.signal(signal.SIGINT, signal_handler)
		signal.pause()
		state0.join()
		if displayGui:
			guiThread.join()

		interfaces.destroyProxies()
	except:
		state0.stop()
		if displayGui:
			gui.close()
			guiThread.join()

		state0.join()
		interfaces.destroyProxies()
		sys.exit(1)
