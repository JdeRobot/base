#!/usr/bin/python
# -*- coding: utf-8 -*-

import Ice
import easyiceconfig as EasyIce
import sys, signal
sys.path.append('/usr/local/share/jderobot/python/visualHFSM_py')
import traceback, threading, time
from automatagui import AutomataGui, QtGui, GuiSubautomata

from jderobot import LaserPrx
from jderobot import MotorsPrx

class Automata():

	def __init__(self):
		self.lock = threading.Lock()
		self.displayGui = False
		self.StatesSub1 = [
			"GoStraight",
			"TurnRight",
			"MoveLeft",
			"MoveRight",
		]

		self.sub1 = "GoStraight"
		self.run1 = True

	def get_min_distance(self):
		laser_data = self.laser_sensor.getLaserData()
		min_dist = 100000
		for i in range(laser_data.numLaser):
			if i < 5:
				continue
			avg_dist = 0
			for j in range(5):
				avg_dist += laser_data.distanceData[i-j]
			avg_dist = avg_dist / 5.0
			if avg_dist < min_dist:
				min_dist = avg_dist
		
		print('min_dist:' + str(min_dist))
		return min_dist
	
	
	def get_left_distance(self):
		laser_data = self.laser_sensor.getLaserData()
		avg_dist = 0
		for i in range(10):
			avg_dist += laser_data.distanceData[laser_data.numLaser-i]
		avg_dist = avg_dist / 10.0
		
		print('avg_dist:'+str(avg_dist))
		return avg_dist
	def startThreads(self):
		self.t1 = threading.Thread(target=self.subautomata1)
		self.t1.start()

	def createAutomata(self):
		guiSubautomataList = []

		# Creating subAutomata1
		guiSubautomata1 = GuiSubautomata(1,0, self.automataGui)

		guiSubautomata1.newGuiNode(1, 0, 107, 124, 1, 'GoStraight')
		guiSubautomata1.newGuiNode(2, 0, 300, 128, 0, 'TurnRight')
		guiSubautomata1.newGuiNode(3, 0, 457, 131, 0, 'MoveLeft')
		guiSubautomata1.newGuiNode(4, 0, 464, 290, 0, 'MoveRight')

		guiSubautomata1.newGuiTransition((107, 124), (300, 128), (190, 59), 1, 1, 2)
		guiSubautomata1.newGuiTransition((300, 128), (457, 131), (369, 59), 2, 2, 3)
		guiSubautomata1.newGuiTransition((457, 131), (464, 290), (553, 205), 4, 3, 4)
		guiSubautomata1.newGuiTransition((464, 290), (457, 131), (390, 209), 5, 4, 3)
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
			if(self.sub1 == "GoStraight"):
				if(self.get_min_distance() < 700):
					self.sub1 = "TurnRight"
					if self.displayGui:
						self.automataGui.notifySetNodeAsActive('TurnRight')

			elif(self.sub1 == "TurnRight"):
				if(self.get_left_distance() < 750):
					self.sub1 = "MoveLeft"
					if self.displayGui:
						self.automataGui.notifySetNodeAsActive('MoveLeft')

			elif(self.sub1 == "MoveLeft"):
				if(self.get_left_distance() < 700):
					self.sub1 = "MoveRight"
					if self.displayGui:
						self.automataGui.notifySetNodeAsActive('MoveRight')

			elif(self.sub1 == "MoveRight"):
				if(self.get_left_distance() > 700):
					self.sub1 = "MoveLeft"
					if self.displayGui:
						self.automataGui.notifySetNodeAsActive('MoveLeft')


			# Actuation if
			if(self.sub1 == "GoStraight"):
				self.my_motors.setV(0.2)
				self.my_motors.setW(0.0)
			elif(self.sub1 == "TurnRight"):
				self.my_motors.setV(0.0)
				self.my_motors.setW(-0.2)
			elif(self.sub1 == "MoveLeft"):
				self.my_motors.setV(0.15)
				self.my_motors.setW(0.15)
			elif(self.sub1 == "MoveRight"):
				self.my_motors.setV(0.15)
				self.my_motors.setW(-0.15)

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

		# Contact to laser_sensor
		laser_sensor = self.ic.propertyToProxy('automata.laser_sensor.Proxy')
		if(not laser_sensor):
			raise Exception('could not create proxy with laser_sensor')
		self.laser_sensor = LaserPrx.checkedCast(laser_sensor)
		if(not self.laser_sensor):
			raise Exception('invalid proxy automata.laser_sensor.Proxy')
		print('laser_sensor connected')

		# Contact to my_motors
		my_motors = self.ic.propertyToProxy('automata.my_motors.Proxy')
		if(not my_motors):
			raise Exception('could not create proxy with my_motors')
		self.my_motors = MotorsPrx.checkedCast(my_motors)
		if(not self.my_motors):
			raise Exception('invalid proxy automata.my_motors.Proxy')
		print('my_motors connected')


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
