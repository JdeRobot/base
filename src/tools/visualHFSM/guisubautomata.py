'''
   Copyright (C) 1997-2016 JDERobot Developers Team
 
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
 
   Authors : Samuel Rey <samuel.rey.escudero@gmail.com> 
 
  '''

from guinode import GuiNode
from guitransition import GuiTransition
import threading


class GuiSubautomata():
	def __init__(self, id, idNodeFather, automataGui):
		self.id = id
		self.idNodeFather = idNodeFather
		self.automataGui = automataGui
		self.activeNode = ""
		self.nodeList = []
		self.transList = []


	def newGuiNode(self, id, idSubSon, x, y, isInit, name):
		gnode = GuiNode(id, idSubSon, self, x, y, isInit, name)
		self.nodeList.append(gnode)


	def newGuiTransition(self, orig, dest, midp, idTrans, idOrig, idDest):
		transition = GuiTransition(orig, dest, midp, idTrans, idOrig, idDest)
		self.transList.append(transition)


	def show(self):
		for node in self.nodeList:
			node.show()

		for transition in self.transList:
			transition.show()


	def hide(self):
		for node in self.nodeList:
			node.hide()

		for transition in self.transList:
			transition.hide()


	def getNode(self, id):
		for node in self.nodeList:
			if node.id == id:
				return node
		return None


	def getNodeByName(self, name):
		for node in self.nodeList:
			if node.name == name:
				return node
		return None


	def setActiveNode(self, nodeName):
		self.activeNode = nodeName


	def getActiveNode(self):
		return self.activeNode


	def drawCopy(self, view, windowId):
		for node in self.nodeList:
			nodeAux = node.createCopy(windowId)
			nodeAux.draw(view)
			if nodeAux.color == "green":
				nodeAux.setColor("green")
				
		for transition in self.transList:
			transAux = transition.createCopy(windowId)
			transAux.draw(view)


	def removeCopy(self, windowId):
		for node in self.nodeList:
			node.removeCopy(windowId)
		for transition in self.transList:
			transition.removeCopy(windowId)