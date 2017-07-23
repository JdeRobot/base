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

from PyQt5.QtWidgets import QGraphicsEllipseItem, QGraphicsObject, QGraphicsItem
from PyQt5.QtCore import Qt, pyqtSignal, QRectF
from PyQt5.QtGui import QBrush, QPen
from . import idtextboxgraphicsitem

# CONST
NODE_WIDTH = 40
INIT_WIDTH = 30
PEN_NORMAL_WIDTH = 1
PEN_FOCUS_WIDTH = 3


class StateGraphicsItem(QGraphicsObject):
    posChanged = pyqtSignal('QGraphicsItem')
    stateNameChanged = pyqtSignal('QGraphicsItem')

    stateTextEditStarted = pyqtSignal()
    stateTextEditFinished = pyqtSignal()

    def __init__(self, id, x, y, initial, name='state'):
        super().__init__()

        self.id = id
        self.name = name
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        # self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setAcceptDrops(True)

        # position of the graphics item on the scene
        self.setPos(x, y)

        self.dragging = False

        # create an ellipse
        self.ellipse = QGraphicsEllipseItem(-NODE_WIDTH / 2, -NODE_WIDTH / 2, NODE_WIDTH, NODE_WIDTH, self)
        brush = QBrush(Qt.SolidPattern)
        brush.setColor(Qt.blue)
        self.ellipse.setBrush(brush)

        self.textGraphics = idtextboxgraphicsitem.IdTextBoxGraphicsItem(self.name, self)
        textWidth = self.textGraphics.boundingRect().width()
        self.textGraphics.setPos(-textWidth / 2, NODE_WIDTH - (NODE_WIDTH / 2) + 5)
        self.textGraphics.textChanged.connect(self.nameChanged)
        self.textGraphics.textEditStarted.connect(self.textEditStarted)
        self.textGraphics.textEditFinished.connect(self.textEditFinished)

        self.initGraphics = None
        # this should run just after other visuals to make sure that it is on the top of other visuals
        self.setInitial(initial)

        # the list of transition that state is the origin for
        self.transitions = []

        # states that is the container for them
        self.childStates = []

        self.code = ""


    def isInitial(self):
        return self.initial

    def setInitial(self, initial):
        self.initial = initial
        if self.initial:
            if self.initGraphics is None:
                self.initGraphics = QGraphicsEllipseItem(-INIT_WIDTH / 2, -INIT_WIDTH / 2, INIT_WIDTH, INIT_WIDTH, self)
        else:
            if self.initGraphics is not None:
                self.initGraphics.setParentItem(None)

    def hoverEnterEvent(self, event):
        myPen = QPen(Qt.SolidLine)
        myPen.setWidth(PEN_FOCUS_WIDTH)
        self.ellipse.setPen(myPen)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        myPen = QPen(Qt.SolidLine)
        myPen.setWidth(PEN_NORMAL_WIDTH)
        self.ellipse.setPen(myPen)
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = True
        super().mousePressEvent(qGraphicsSceneMouseEvent)

    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = False

        super().mouseReleaseEvent(qGraphicsSceneMouseEvent)

    def mouseMoveEvent(self, qGraphicsSceneMouseEvent):
        if self.dragging:
            self.posChanged.emit(self)
        super().mouseMoveEvent(qGraphicsSceneMouseEvent)

    def boundingRect(self):
        return self.ellipse.boundingRect()

    def paint(self, qPainter, qStyleOptionGraphicsItem, qWidget_widget=None):
        pass

    def nameChanged(self, newName):
        print('text changed')
        self.name = newName
        textWidth = self.textGraphics.boundingRect().width()
        # reposition to center the text
        self.textGraphics.setPos(-textWidth / 2, NODE_WIDTH - (NODE_WIDTH / 2) + 5)
        self.stateNameChanged.emit(self)

    def textEditStarted(self):
        self.stateTextEditStarted.emit()

    def textEditFinished(self):
        self.stateTextEditFinished.emit()

    def addTransition(self, transition):
        self.transitions.append(transition)

    def getTransitions(self):
        return self.transitions

    def getChildren(self):
        return self.childStates

    def addChild(self, child):
        if child not in self.childStates:
            self.childStates.append(child)



    # def mouseDoubleClickEvent(self, event):
    # 	# self.state.notifyChangeCurrentSubautomata(self.state.idSubautSon)
    # 	pass


    #
    # def __init__(self, id, idSubSon, subautomata, x, y, isInit, name, windowId=0):
    # 	self.id = id
    # 	self.idSubautSon = idSubSon
    # 	self.subautomata = subautomata
    # 	self.x = x - NODE_WIDTH/2
    # 	self.y = y - NODE_WIDTH/2
    # 	self.isInit = isInit
    # 	self.name = name
    # 	self.copies = []
    # 	self.windowId = windowId
    # 	self.color = "blue"
    #
    # 	#CREATE GUI ELEMENTS
    # 	self.ellipse = self.State(self, self.x, self.y, NODE_WIDTH)
    # 	self.text = QtGui.QGraphicsSimpleTextItem(self.name)
    # 	self.text.setPos(self.x, self.y+NODE_WIDTH)
    # 	if self.isInit:
    # 		self.ellipseInit = QtGui.QGraphicsEllipseItem(self.x+5, self.y+5,
    # 								INIT_WIDTH, INIT_WIDTH)
    # 	else:
    # 		self.ellipseInit = None
    # 	self.paint("blue", PEN_NORMAL_WIDTH)
    #
    #
    # def getIdNodeFather(self):
    # 	return self.subautomata.idNodeFather
    #
    # def show(self):
    # 	self.ellipse.show()
    # 	self.text.show()
    # 	if self.isInit:
    # 		self.ellipseInit.show()
    #
    #
    # def hide(self):
    # 	self.ellipse.hide()
    # 	self.text.hide()
    # 	if self.isInit:
    # 		self.ellipseInit.hide()
    #
    #
    # def setWidth(self, width):
    # 	pen = QtGui.QPen(QtGui.QColor("black"), width)
    # 	self.ellipse.setPen(pen)
    # 	if self.isInit:
    # 		self.ellipseInit.setPen(pen)
    #
    #
    # def setColor(self, color):
    # 	self.paint(color, PEN_NORMAL_WIDTH)
    #
    #
    # def paint(self, color, width):
    # 	pen = QtGui.QPen(QtGui.QColor("black"), width)
    # 	brush = QtGui.QBrush(QtCore.Qt.SolidPattern)
    # 	brush.setColor(QtGui.QColor(color))
    # 	self.ellipse.setPen(pen)
    # 	self.ellipse.setBrush(brush)
    # 	self.color = color
    # 	for copy in self.copies:
    # 		copy.paint(color, width)
    #
    #
    # def notifyChangeCurrentSubautomata(self, idNewSub):
    # 	if self.windowId == 0:
    # 		if idNewSub != 0:
    # 			self.setWidth(PEN_NORMAL_WIDTH)
    # 			self.subautomata.automataGui.changeCurrentSubautomata(idNewSub)
    # 		else:
    # 			print("This node does not have any subautomata son")
    #
    #
    # def draw(self, view):
    # 	view.addItem(self.ellipse)
    # 	view.addItem(self.text)
    # 	if self.isInit:
    # 		view.addItem(self.ellipseInit)
    #
    #
    # def createCopy(self, windowId):
    # 	x = self.x + NODE_WIDTH/2
    # 	y = self.y + NODE_WIDTH/2
    # 	nodeAux = GuiNode(self.id, self.idSubautSon, self.subautomata,
    # 				 		x, y, self.isInit, self.name, windowId)
    # 	nodeAux.color = self.color
    # 	self.copies.append(nodeAux)
    # 	return self.copies[-1]
    #
    #
    # def removeCopy(self, windowId):
    # 	for copy in self.copies:
    # 		if copy.windowId == windowId:
    # 			self.copies.remove(copy)
