from .transitiontype import TransitionType
from .guitransition import TransitionGraphicsItem
from PyQt5.QtCore import QPointF

class Transition:
    def __init__(self, id, name, origin, dest):
        self.id = id
        self.name = name
        self.transitionType = TransitionType.TEMPORAL
        self.code = ''
        self.temporalTime = 0
        self.condition = ''
        self.x = 0
        self.y = 0
        self.isPosChanged = False

        # set transitions on the state
        self.origin = origin
        self.origin.addOriginTransition(self)
        self.destination = dest
        self.destination.addDestTransition(self)

        self.graphicsItem = None

    def getGraphicsItem(self):
        if self.graphicsItem == None:
            self.graphicsItem = TransitionGraphicsItem(self)
            self.graphicsItem.posChanged.connect(self.posChanged)

        if self.isPosChanged:
            self.graphicsItem.updateMiddlePoints(QPointF(self.x, self.y))
            self.graphicsItem.createMiddleHandle()

        return self.graphicsItem

    def resetGraphicsItem(self):
        self.graphicsItem = None

    def getTemporalTime(self):
        return self.temporalTime

    def setTemporalTime(self, time):
        self.temporalTime = time

    def getCondition(self):
        return self.condition

    def setCondition(self, cond):
        self.condition = cond

    def getType(self):
        return self.transitionType

    def setType(self, type):
        if type == TransitionType.TEMPORAL or type == TransitionType.CONDITIONAL:
            self.transitionType = type

    def getCode(self):
        return self.code

    def setCode(self, code):
        self.code = code

    def posChanged(self, tranItem):
        self.isPosChanged = True
        self.x = tranItem.midPointX
        self.y = tranItem.midPointY

