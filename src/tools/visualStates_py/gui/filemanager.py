
from xml.dom import minidom
from .guistate import StateGraphicsItem
from .guitransition import TransitionGraphicsItem
from PyQt5.QtCore import QPointF

class FileManager():
    def __init__(self):
        print('file manager')

        self.fileName = ""


    def getFileName(self):
        return self.fileName

    def setFileName(self, name):
        self.fileName = name

    def save(self, rootState):
        doc = minidom.Document()
        root = doc.createElement('VisualStates')
        doc.appendChild(root)

        root.appendChild(self.createDocFromState(rootState, doc))
        xmlStr = doc.toprettyxml(indent='  ')
        with open(self.fileName, 'w') as f:
            f.write(xmlStr)


    def createDocFromState(self, state, doc):
        stateElement = doc.createElement('state')
        stateElement.setAttribute('initial', str(state.isInitial()))
        stateElement.setAttribute('id', str(state.id))
        posxElement = doc.createElement('posx')
        posxElement.appendChild(doc.createTextNode(str(state.pos().x())))
        posyElement = doc.createElement('posy')
        posyElement.appendChild(doc.createTextNode(str(state.pos().y())))
        stateElement.appendChild(posxElement)
        stateElement.appendChild(posyElement)
        nameElement = doc.createElement('name')
        nameElement.appendChild(doc.createTextNode(state.name))
        stateElement.appendChild(nameElement)

        # create transitions of the state
        for transition in state.getTransitions():
            tranElement = doc.createElement('transition')
            tranElement.setAttribute('id', str(transition.id))

            tposxElement = doc.createElement('posx')
            tposxElement.appendChild(doc.createTextNode(str(transition.midPointX)))
            tranElement.appendChild(tposxElement)
            tposyElement = doc.createElement('posy')
            tposyElement.appendChild(doc.createTextNode(str(transition.midPointY)))
            tranElement.appendChild(tposyElement)
            nameElement = doc.createElement('name')
            nameElement.appendChild(doc.createTextNode(transition.name))
            tranElement.appendChild(nameElement)
            originElement = doc.createElement('origin')
            originElement.appendChild(doc.createTextNode(str(transition.origin.id)))
            tranElement.appendChild(originElement)
            destinElement = doc.createElement('destination')
            destinElement.appendChild(doc.createTextNode(str(transition.destination.id)))
            tranElement.appendChild(destinElement)

            stateElement.appendChild(tranElement)

        for child in state.getChildren():
            stateElement.appendChild(self.createDocFromState(child, doc))

        return stateElement

    def open(self, fileName):
        self.setFileName(fileName)
        allTransitionNodes = []
        doc = minidom.parse(self.fileName)
        rootNode = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('state')[0]
        rootState = StateGraphicsItem(0, 0, 0, True, 'root')
        self.parseStateNode(rootNode, rootState)

        return rootState

    def parseStateNode(self, node, rootState):
        allTransitionNodes = []
        stateNodes = node.getElementsByTagName('state')
        statesById = {}
        for sNode in stateNodes:
            id = 0
            initial = False
            for (name, value) in sNode.attributes.items():
                if name == 'id':
                    id = int(value)
                elif name == 'initial':
                    initial = (value == 'True')
            name = sNode.getElementsByTagName('name')[0].childNodes[0].nodeValue
            print('read name:' + name)
            posx = float(sNode.getElementsByTagName('posx')[0].childNodes[0].nodeValue)
            posy = float(sNode.getElementsByTagName('posy')[0].childNodes[0].nodeValue)
            state = StateGraphicsItem(id, posx, posy, initial, name)
            statesById[state.id] = state
            rootState.addChild(state)
            transitionNodes = sNode.getElementsByTagName('transition')
            for tNode in transitionNodes:
                allTransitionNodes.append(tNode)

            self.parseStateNode(sNode, state)


        # create transitions
        for tNode in allTransitionNodes:
            id = 0
            for (name, value) in tNode.attributes.items():
                if name == 'id':
                    id = int(value)
            name = tNode.getElementsByTagName('name')[0].childNodes[0].nodeValue
            posx = float(tNode.getElementsByTagName('posx')[0].childNodes[0].nodeValue)
            posy = float(tNode.getElementsByTagName('posy')[0].childNodes[0].nodeValue)
            originId = int(tNode.getElementsByTagName('origin')[0].childNodes[0].nodeValue)
            destinationId = int(tNode.getElementsByTagName('destination')[0].childNodes[0].nodeValue)
            tran = TransitionGraphicsItem(statesById[originId], statesById[destinationId], id, name)
            tran.updateMiddlePoints(QPointF(posx, posy))
            tran.createMiddleHandle()






