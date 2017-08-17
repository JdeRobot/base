
from xml.dom import minidom
from PyQt5.QtCore import QPointF
from .state import State
import os

class FileManager():
    def __init__(self):
        self.fullPath = ""

    def getFullPath(self):
        return self.fullPath

    def setFullPath(self, path):
        self.fullPath = path

    def save(self, rootState):
        doc = minidom.Document()
        root = doc.createElement('VisualStates')
        doc.appendChild(root)

        root.appendChild(self.createDocFromState(rootState, doc))
        xmlStr = doc.toprettyxml(indent='  ')
        with open(self.fullPath+'.xml', 'w') as f:
            f.write(xmlStr)

    def createDocFromState(self, state, doc):
        stateElement = state.createElement(doc)
        return stateElement

    def open(self, fullPath):
        projectName = fullPath[0:fullPath.rfind('.')]
        self.setFullPath(projectName)
        doc = minidom.parse(fullPath)
        rootNode = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('state')[0]
        rootState = State(0, 'root', True)
        rootState.parse(rootNode)

        return rootState

    def hasFile(self):
        return len(self.fullPath) > 0

    def getFileName(self):
        name = ''
        if self.fullPath.rfind(os.sep) >= 0:
            name = self.fullPath[self.fullPath.rfind(os.sep)+1:len(self.fullPath)]

        # remove the file extension
        if len(name) > 0:
            name = name[0:name.rfind('.')]

        return name

    def getPath(self):
        path = ''
        if self.fullPath.rfind(os.sep) >= 0:
            path = self.fullPath[0:self.fullPath.rfind(os.sep)]
        return path






