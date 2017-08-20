from .guistate import StateGraphicsItem
from .transitiontype import TransitionType
from .transition import Transition
from xml.dom.minidom import Node

class State:
    def __init__(self, id, name, initial, parent=None):
        self.parent = parent
        self.id = id
        self.name = name
        self.code = ''
        self.functions = ''
        self.variables = ''
        self.timeStepDuration = 100
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

    def parse(self, stateElement):

        # parse attributes of the state
        for (name, value) in stateElement.attributes.items():
            if name == 'id':
                self.id = int(value)
            elif name == 'initial':
                self.initial = (value == 'True')

        self.name = stateElement.getElementsByTagName('name')[0].childNodes[0].nodeValue
        print('read name:' + self.name)
        self.x = float(stateElement.getElementsByTagName('posx')[0].childNodes[0].nodeValue)
        self.y = float(stateElement.getElementsByTagName('posy')[0].childNodes[0].nodeValue)

        # optinal state tags
        if len(stateElement.getElementsByTagName('code')[0].childNodes) > 0:
            self.code = stateElement.getElementsByTagName('code')[0].childNodes[0].nodeValue

        if len(stateElement.getElementsByTagName('functions')[0].childNodes) > 0:
            self.functions = stateElement.getElementsByTagName('functions')[0].childNodes[0].nodeValue

        if len(stateElement.getElementsByTagName('timestep')[0].childNodes) > 0:
            self.timeStepDuration = int(stateElement.getElementsByTagName('timestep')[0].childNodes[0].nodeValue)

        # recursive child state parsing
        allChildTransitions = []
        statesById = {}
        stateTransitions = []
        for childNode in stateElement.childNodes:
            if childNode.nodeType == Node.ELEMENT_NODE:
                if childNode.tagName == 'state':
                    childState = State(0, 'state', False, self)
                    transitionNodes = childState.parse(childNode)
                    print('add child:' + childState.name + ' to parent:' + self.name)
                    self.addChild(childState)
                    statesById[childState.id] = childState
                    allChildTransitions = allChildTransitions + transitionNodes
                elif childNode.tagName == 'transition':
                    stateTransitions.append(childNode)

        # wire transitions with the states after all the child states are parsed
        for tranNode in allChildTransitions:
            transition = Transition(0, 'transition')
            transition.parse(tranNode, statesById)

        # return transitions of the state to be able to wire after all states are created
        return stateTransitions

    def createElement(self, doc, parentElement=None):
        stateElement = doc.createElement('state')
        stateElement.setAttribute('initial', str(self.initial))
        stateElement.setAttribute('id', str(self.id))
        posxElement = doc.createElement('posx')
        posxElement.appendChild(doc.createTextNode(str(self.x)))
        posyElement = doc.createElement('posy')
        posyElement.appendChild(doc.createTextNode(str(self.y)))
        stateElement.appendChild(posxElement)
        stateElement.appendChild(posyElement)
        nameElement = doc.createElement('name')
        nameElement.appendChild(doc.createTextNode(self.name))
        stateElement.appendChild(nameElement)
        codeElement = doc.createElement('code')
        codeElement.appendChild(doc.createTextNode(self.code))
        stateElement.appendChild(codeElement)
        functionsElement = doc.createElement('functions')
        functionsElement.appendChild(doc.createTextNode(self.functions))
        stateElement.appendChild(functionsElement)
        timeElement = doc.createElement('timestep')
        timeElement.appendChild(doc.createTextNode(str(self.timeStepDuration)))
        stateElement.appendChild(timeElement)

        # create transition elements
        for tran in self.getOriginTransitions():
            tranElement = tran.createElement(doc)
            stateElement.appendChild(tranElement)

        for child in self.getChildren():
            child.createElement(doc, stateElement)

        if parentElement is not None:
            parentElement.appendChild(stateElement)

        return stateElement

    def getCode(self):
        return self.code

    def setCode(self, code):
        self.code = code

    def getFunctions(self):
        return self.functions

    def setFunctions(self, functions):
        self.functions = functions

    def getVariables(self):
        return self.variables

    def setVariables(self, vars):
        self.variables = vars

    def getTimeStep(self):
        return self.timeStepDuration

    def setTimeStep(self, timestep):
        self.timeStepDuration = timestep

    def getInitialChild(self):
        for child in self.getChildren():
            if child.initial:
                return child

        return None

    def setInitial(self, initial):
        self.initial = initial

    def getChildrenTransitions(self):
        childTransitions = []
        for child in self.getChildren():
            childTransitions += child.getOriginTransitions()
        return childTransitions
