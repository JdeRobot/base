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

from PyQt4 import QtGui
from PyQt4 import QtCore
import threading

#CONST
NODE_WIDTH = 40
INIT_WIDTH = 30
PEN_NORMAL_WIDTH = 1
PEN_FOCUS_WIDTH = 3


class GuiNode():
	class State(QtGui.QGraphicsEllipseItem):
		def __init__(self, state, x, y, width):
			QtGui.QGraphicsEllipseItem.__init__(self, x, y, width, width)
			self.setAcceptHoverEvents(True)
			self.state = state


		def hoverEnterEvent(self, event):
			self.state.setWidth(PEN_FOCUS_WIDTH)


		def hoverLeaveEvent(self, event):
			self.state.setWidth(PEN_NORMAL_WIDTH)


		def mouseDoubleClickEvent(self, event):
			self.state.notifyChangeCurrentSubautomata(self.state.idSubautSon)


	def __init__(self, id, idSubSon, subautomata, x, y, isInit, name, windowId=0):
		self.id = id
		self.idSubautSon = idSubSon
		self.subautomata = subautomata
		self.x = x - NODE_WIDTH/2
		self.y = y - NODE_WIDTH/2
		self.isInit = isInit
		self.name = name
		self.copies = []
		self.windowId = windowId
		self.color = "blue"

		#CREATE GUI ELEMENTS
		self.ellipse = self.State(self, self.x, self.y, NODE_WIDTH)
		self.text = QtGui.QGraphicsSimpleTextItem(self.name)
		self.text.setPos(self.x, self.y+NODE_WIDTH)
		if self.isInit:
			self.ellipseInit = QtGui.QGraphicsEllipseItem(self.x+5, self.y+5, 
									INIT_WIDTH, INIT_WIDTH)
		else:
			self.ellipseInit = None
		self.paint("blue", PEN_NORMAL_WIDTH)


	def getIdNodeFather(self):
		return self.subautomata.idNodeFather

	def show(self):
		self.ellipse.show()
		self.text.show()
		if self.isInit:
			self.ellipseInit.show()


	def hide(self):
		self.ellipse.hide()
		self.text.hide()
		if self.isInit:
			self.ellipseInit.hide()


	def setWidth(self, width):
		pen = QtGui.QPen(QtGui.QColor("black"), width)
		self.ellipse.setPen(pen)
		if self.isInit:
			self.ellipseInit.setPen(pen)


	def setColor(self, color):
		self.paint(color, PEN_NORMAL_WIDTH)
		

	def paint(self, color, width):
		pen = QtGui.QPen(QtGui.QColor("black"), width)
		brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
		brush.setColor(QtGui.QColor(color))
		self.ellipse.setPen(pen)
		self.ellipse.setBrush(brush)
		self.color = color
		for copy in self.copies:
			copy.paint(color, width)


	def notifyChangeCurrentSubautomata(self, idNewSub):
		if self.windowId == 0:
			if idNewSub != 0:
				self.setWidth(PEN_NORMAL_WIDTH)
				self.subautomata.automataGui.changeCurrentSubautomata(idNewSub)
			else:
				print "This node does not have any subautomata son"


	def draw(self, view):
		view.addItem(self.ellipse)
		view.addItem(self.text)
		if self.isInit:
			view.addItem(self.ellipseInit)


	def createCopy(self, windowId):
		x = self.x + NODE_WIDTH/2
		y = self.y + NODE_WIDTH/2
		nodeAux = GuiNode(self.id, self.idSubautSon, self.subautomata,
					 		x, y, self.isInit, self.name, windowId)
		nodeAux.color = self.color
		self.copies.append(nodeAux)
		return self.copies[-1]


	def removeCopy(self, windowId):
		for copy in self.copies:
			if copy.windowId == windowId:
				self.copies.remove(copy)
