
from xml.dom import minidom
from PyQt5.QtCore import QPointF
from .state import State

class FileManager():
    def __init__(self):
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
        stateElement = state.createElement(doc)
        return stateElement

    def open(self, fileName):
        self.setFileName(fileName)
        allTransitionNodes = []
        doc = minidom.parse(self.fileName)
        rootNode = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('state')[0]
        rootState = State(0, 'root', True)
        rootState.parse(rootNode)

        return rootState




