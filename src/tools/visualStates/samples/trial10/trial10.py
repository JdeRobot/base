#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys, threading, time
import easyiceconfig as EasyIce
sys.path.append("/opt/jderobot/lib/python2.7")
sys.path.append("/opt/jderobot/share/jderobot/python3/visualStates_py")
from codegen.state import State
from codegen.temporaltransition import TemporalTransition
from codegen.conditionaltransition import ConditionalTransition
from codegen.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication


from jderobot import MotorsPrx

class State0(State):
	def runCode(self):
		pass


class State1(State):
	def runCode(self):
		pass


class State2(State):
	def runCode(self):
		interfaces->myMotors->setV(0.0);


class State3(State):
	def runCode(self):
		interfaces->myMotors->setV(0.2);


class State4(State):
	def runCode(self):
		interfaces->myMotors->setV(0.0);


class Tran1(TemporalTransition):

	def runCode(self):
		pass

class Tran5(TemporalTransition):

	def runCode(self):
		pass

class Tran2(TemporalTransition):

	def runCode(self):
		pass

class Tran4(TemporalTransition):

	def runCode(self):
		pass

class Interfaces():
	def __init__(self):
		self.ic = None
		self.myMotors = None
		self.connectProxies()

	def connectProxies(self):
		self.ic = EasyIce.initialize(sys.argv)
		self.myMotors = self.ic.propertyToProxy("automata.myMotors.Proxy")
		if not self.myMotors:
			raise Exception("could not create proxy with name:myMotors")
		self.myMotors = MotorsPrx.checkedCast(self.myMotors)
		if not self.myMotors:
			raise Exception("invalid proxy automata.myMotors.Proxy")

	def destroyProxies(self):
		if self.ic is not None:
			self.ic.destroy()

displayGui = False
guiThread = None
gui = None

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
		gui.addState(1, "moving", True, 781.0, 764.0, 0)
		gui.addState(2, "stop", False, 944.0, 938.0, 0)
		gui.addState(3, "state 3", True, 817.0, 800.0, 1)
		gui.addState(4, "state 4", False, 984.0, 979.0, 1)

		gui.addTransition(1, "10secs", 1, 2, 803.5, 929.0)
		gui.addTransition(5, "transition 5", 2, 1, 903.5, 800.0)
		gui.addTransition(2, "transition 2", 3, 4, 865.5, 977.5)
		gui.addTransition(4, "transition 4", 4, 3, 969.5, 832.5)

	if displayGui:
		gui.emitActiveStateById(0)

	state0 = State0(0, True, interfaces, 100, None, gui)
	state1 = State1(1, True, interfaces, 100, state0, gui)
	state2 = State2(2, False, interfaces, 100, state0, gui)
	state3 = State3(3, True, interfaces, 100, state1, gui)
	state4 = State4(4, False, interfaces, 100, state1, gui)

	tran1 = Tran1(1, 2, 10000)
	state1.addTransition(tran1)

	tran5 = Tran5(5, 1, 5000)
	state2.addTransition(tran5)

	tran2 = Tran2(2, 4, 1000)
	state3.addTransition(tran2)

	tran4 = Tran4(4, 3, 1000)
	state4.addTransition(tran4)

	try:
		state0.startThread()
		state1.startThread()
		state0.join()
		state1.join()
		interfaces.destroyProxies()
		sys.exit(0)
	except:
		state0.stop()
		state1.stop()
		if displayGui:
			gui.close()
			guiThread.join()

		state0.join()
		state1.join()
		interfaces.destroyProxies()
		sys.exit(1)
