from .guistate import StateGraphicsItem

class State:
    def __init__(self, id, name, initial, parent=None):
        self.parent = parent
        self.id = id
        self.name = name
        self.code = ''
        self.x = 0
        self.y = 0
        self.initial = initial
        self.children = []
        self.originTransitions = []
        self.destTransitions = []

        self.graphicsItem = None

    def setPos(self, x, y):
        self.x = x
        self.y = y

    def addChild(self, child):
        if child not in self.children:
            self.children.append(child)

    def removeChild(self, child):
        if child in self.children:
            self.children.remove(child)

    def getChildren(self):
        return self.children

    def getOriginTransitions(self):
        return self.originTransitions

    def addOriginTransition(self, tran):
        if tran not in self.originTransitions:
            self.originTransitions.append(tran)

    def removeOriginTransition(self, tran):
        if tran in self.originTransitions:
            self.originTransitions.remove(tran)

    def getDestTransitions(self):
        return self.destTransitions

    def addDestTransition(self, tran):
        if tran not in self.destTransitions:
            self.destTransitions.append(tran)

    def removeDestTransition(self, tran):
        if tran in self.destTransitions:
            self.destTransitions.remove(tran)

    def getGraphicsItem(self):
        if self.graphicsItem == None:
            self.graphicsItem = StateGraphicsItem(self)
            self.graphicsItem.posChanged.connect(self.posChanged)
        return self.graphicsItem

    def resetGraphicsItem(self):
        self.graphicsItem = None

    def posChanged(self, stateItem):
        scenePos = stateItem.scenePos()
        self.x = scenePos.x()
        self.y = scenePos.y()

    # creates a new copy of the state without children and transitions
    def getNewCopy(self):
        copy = State(self.id, self.name, False, self.parent)
        copy.code = self.code
        copy.x = self.x
        copy.y = self.y
        return copy
