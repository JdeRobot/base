#!/usr/bin/python
# -*- coding: utf-8 -*-

import Ice
import easyiceconfig as EasyIce
import sys, signal
sys.path.append('/opt/jderobot/share/jderobot/python/visualStates_py')
import traceback, threading, time
from automatagui import AutomataGui, QtGui, GuiSubautomata

from jderobot import MotorsPrx

class Automata():

	def __init__(self):
		self.lock = threading.Lock()
		self.displayGui = False
		self.StatesSub1 = [
			"state1",
			"state2",
		]

		self.StatesSub2 = [
			"state11",
			"state11_ghost",
			"state12",
			"state12_ghost",
		]

		self.sub1 = "state1"
		self.run1 = True
		self.sub2 = "state11_ghost"
		self.run2 = True

	int functions(int a, int b) {
		return a+b;
	}
	def startThreads(self):
		self.t1 = threading.Thread(target=self.subautomata1)
		self.t1.start()
		self.t2 = threading.Thread(target=self.subautomata2)
		self.t2.start()

	def createAutomata(self):
		guiSubautomataList = []

		# Creating subAutomata1
		guiSubautomata1 = GuiSubautomata(1,0, self.automataGui)

		guiSubautomata1.newGuiNode(1, 2, 118, 114, 1, 'state1')
		guiSubautomata1.newGuiNode(2, 0, 354, 313, 0, 'state2')

		guiSubautomata1.newGuiTransition((118, 114), (354, 313), (299, 156), 1, 1, 2)
		guiSubautomata1.newGuiTransition((354, 313), (118, 114), (171, 304), 3, 2, 1)
		guiSubautomataList.append(guiSubautomata1)

		# Creating subAutomata2
		guiSubautomata2 = GuiSubautomata(2,1, self.automataGui)

		guiSubautomata2.newGuiNode(3, 0, 147, 123, 1, 'state11')
		guiSubautomata2.newGuiNode(4, 0, 407, 347, 0, 'state12')

		guiSubautomata2.newGuiTransition((147, 123), (407, 347), (277, 235), 2, 3, 4)
		guiSubautomataList.append(guiSubautomata2)


		return guiSubautomataList

	def shutDown(self):
		self.run1 = False
		self.run2 = False

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
		cycle = 200
		t_activated = False
		t_fin = 0

		int variables = 10;

		while(self.run1):
			totala = time.time() * 1000000

			# Evaluation if
			if(self.sub1 == "state1"):
				if(not t_activated):
					t_ini = time.time()
					t_activated = True
				else:
					t_fin = time.time()
					secs = t_fin - t_ini
					if(secs > 3):
						self.sub1 = "state2"
						t_activated = False
						int transition_code = 12;
						if self.displayGui:
							self.automataGui.notifySetNodeAsActive('state2')

			elif(self.sub1 == "state2"):
				if(not t_activated):
					t_ini = time.time()
					t_activated = True
				else:
					t_fin = time.time()
					secs = t_fin - t_ini
					if(secs > 3):
						self.sub1 = "state1"
						t_activated = False
						if self.displayGui:
							self.automataGui.notifySetNodeAsActive('state1')


			# Actuation if
			if(self.sub1 == "state1"):
				int state1_code = 10;
			elif(self.sub1 == "state2"):
				int state2_code = 10;

			totalb = time.time() * 1000000
			msecs = (totalb - totala) / 1000;
			if(msecs < 0 or msecs > cycle):
				msecs = cycle
			else:
				msecs = cycle - msecs

			time.sleep(msecs / 1000)
			if(msecs < 33 ):
				time.sleep(33 / 1000);


	def subautomata2(self):
		self.run2 = True
		cycle = 100
		t_activated = False
		t_fin = 0

		t_state11_max = 10


		while(self.run2):
			totala = time.time() * 1000000

			if(self.sub1 == "state1"):
				if ((self.sub2 == "state11_ghost") or (self.sub2 == "state12_ghost")):
					ghostStateIndex = self.StatesSub2.index(self.sub2)
					self.sub2 = self.StatesSub2[ghostStateIndex - 1]
					t_ini = time.time()

				# Evaluation if
				if(self.sub2 == "state11"):
					if(not t_activated):
						t_ini = time.time()
						t_activated = True
					else:
						t_fin = time.time()
						secs = t_fin - t_ini
						if(secs > t_state11_max):
							self.sub2 = "state12"
							t_activated = False
							if self.displayGui:
								self.automataGui.notifySetNodeAsActive('state12')
							t_state11_max = 10


				# Actuation if
				if(self.sub2 == "state11"):
					myMotors->setV(0.2);
				elif(self.sub2 == "state12"):
					myMotors->setV(0.0);
			else:
				if(self.sub2 == "state11"):
					t_state11_max = 10 - (t_fin - t_ini)
					ghostStateIndex = self.StatesSub2.index(self.sub2) + 1
					self.sub2 = self.StatesSub2[ghostStateIndex]
				elif(self.sub2 == "state12"):
					ghostStateIndex = self.StatesSub2.index(self.sub2) + 1
					self.sub2 = self.StatesSub2[ghostStateIndex]

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

		# Contact to myMotors
		myMotors = self.ic.propertyToProxy('automata.myMotors.Proxy')
		if(not myMotors):
			raise Exception('could not create proxy with myMotors')
		self.myMotors = MotorsPrx.checkedCast(myMotors)
		if(not self.myMotors):
			raise Exception('invalid proxy automata.myMotors.Proxy')
		print('myMotors connected')


	def destroyIc(self):
		if(self.ic):
			self.ic.destroy()


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
		self.t2.join()


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
