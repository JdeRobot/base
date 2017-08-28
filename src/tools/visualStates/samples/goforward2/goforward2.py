#!/usr/bin/python
# -*- coding: utf-8 -*-

import easyiceconfig as EasyIce
import jderobotComm as comm
import sys, signal
sys.path.append('/usr/local/share/jderobot/python/visualHFSM_py')
import traceback, threading, time
from automatagui import AutomataGui, QtGui, GuiSubautomata

from jderobot import MotorsPrx

class Automata():

	def __init__(self):
		self.lock = threading.Lock()
		self.displayGui = False
		self.StatesSub1 = [
			"goforward",
		]

		self.sub1 = "goforward"
		self.run1 = True

	def startThreads(self):
		self.t1 = threading.Thread(target=self.subautomata1)
		self.t1.start()

	def createAutomata(self):
		guiSubautomataList = []

		# Creating subAutomata1
		guiSubautomata1 = GuiSubautomata(1,0, self.automataGui)

		guiSubautomata1.newGuiNode(1, 0, 118, 121, 1, 'goforward')

		guiSubautomataList.append(guiSubautomata1)


		return guiSubautomataList

	def shutDown(self):
		self.run1 = False

	def runGui(self):
		app = QtGui.QApplication(sys.argv)
		self.automataGui = AutomataGui()
		self.automataGui.setAutomata(self.createAutomata())
		self.automataGui.loadAutomata()
		self.startThreads()
		self.automataGui.show()
		app.exec_()

	def subautomata1(self):
		self.run1 = True
		cycle = 100
		t_activated = False
		t_fin = 0


		while(self.run1):
			totala = time.time() * 1000000

			# Evaluation if

			# Actuation if
			if(self.sub1 == "goforward"):
				self.my_motors.sendV(0.2)

			totalb = time.time() * 1000000
			msecs = (totalb - totala) / 1000;
			if(msecs < 0 or msecs > cycle):
				msecs = cycle
			else:
				msecs = cycle - msecs

			time.sleep(msecs / 1000)
			if(msecs < 33 ):
				time.sleep(33 / 1000);


	def connectToProxys(self):
		self.ic = EasyIce.initialize(sys.argv)
		self.ic,self.node = comm.init(self.ic)

		# Contact to my_motors
		self.my_motors = comm.getMotorsClient(self.ic, 'automata.my_motors')
		if(not self.my_motors):
			raise Exception('could not create client with my_motors')
		print('my_motors connected')


	def destroyIc(self):
		self.my_motors.stop()
		comm.destroy(self.ic, self.node)

	def start(self):
		if self.displayGui:
			self.guiThread = threading.Thread(target=self.runGui)
			self.guiThread.start()
		else:
			self.startThreads()



	def join(self):
		if self.displayGui:
			self.guiThread.join()
		self.t1.join()


	def readArgs(self):
		for arg in sys.argv:
			splitedArg = arg.split('=')
			if splitedArg[0] == '--displaygui':
				if splitedArg[1] == 'True' or splitedArg[1] == 'true':
					self.displayGui = True
					print('runtime gui enabled')
				else:
					self.displayGui = False
					print('runtime gui disabled')


if __name__ == '__main__':
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	automata = Automata()
	try:
		automata.connectToProxys()
		automata.readArgs()
		automata.start()
		automata.join()

		sys.exit(0)
	except:
		traceback.print_exc()
		automata.destroyIc()
		sys.exit(-1)
