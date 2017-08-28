'''
   Copyright (C) 1997-2017 JDERobot Developers Team

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

   Authors : Okan Asik (asik.okan@gmail.com)

  '''
from xml.dom import minidom
from gui.state import State
import os

class FileManager():
    def __init__(self):
        self.fullPath = ""

    def getFullPath(self):
        return self.fullPath

    def setFullPath(self, path):
        self.fullPath = path

    def save(self, rootState, configs, libraries):
        doc = minidom.Document()
        root = doc.createElement('VisualStates')
        doc.appendChild(root)

        # save config data
        configsElement = doc.createElement('configs')
        for cfg in configs:
            cfgElement = doc.createElement('config')
            serverElement = doc.createElement('server')
            serverElement.appendChild(doc.createTextNode(cfg['serverType']))
            cfgElement.appendChild(serverElement)
            nameElement = doc.createElement('name')
            nameElement.appendChild(doc.createTextNode(cfg['name']))
            cfgElement.appendChild(nameElement)
            topicElement = doc.createElement('topic')
            topicElement.appendChild(doc.createTextNode(cfg['topic']))
            cfgElement.appendChild(topicElement)
            proxyNameElement = doc.createElement('proxyname')
            proxyNameElement.appendChild(doc.createTextNode(cfg['proxyName']))
            cfgElement.appendChild(proxyNameElement)
            ipElement = doc.createElement('ip')
            ipElement.appendChild(doc.createTextNode(cfg['ip']))
            cfgElement.appendChild(ipElement)
            portElement = doc.createElement('port')
            portElement.appendChild(doc.createTextNode(cfg['port']))
            cfgElement.appendChild(portElement)
            interfaceElement = doc.createElement('interface')
            interfaceElement.appendChild(doc.createTextNode(cfg['interface']))
            cfgElement.appendChild(interfaceElement)
            configsElement.appendChild(cfgElement)
        root.appendChild(configsElement)

        # save libraries
        libraryElement = doc.createElement('libraries')
        for lib in libraries:
            libElement = doc.createElement('library')
            libElement.appendChild(doc.createTextNode(lib))
            libraryElement.appendChild(libElement)
        root.appendChild(libraryElement)

        root.appendChild(self.createDocFromState(rootState, doc))
        xmlStr = doc.toprettyxml(indent='  ')
        with open(self.fullPath, 'w') as f:
            f.write(xmlStr)

    def createDocFromState(self, state, doc):
        stateElement = state.createElement(doc)
        return stateElement

    def open(self, fullPath):
        self.setFullPath(fullPath)
        doc = minidom.parse(fullPath)
        rootNode = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('state')[0]
        rootState = State(0, 'root', True)
        rootState.parse(rootNode)

        configs = []

        # parse configs
        configsElements = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('configs')
        if len(configsElements) > 0:
            configElements = configsElements[0].getElementsByTagName('config')
            cfg = None
            if len(configElements) > 0:
                cfg = {}
            for cfgElement in configElements:
                if len(cfgElement.getElementsByTagName('server')[0].childNodes) > 0:
                    cfg['serverType'] = cfgElement.getElementsByTagName('server')[0].childNodes[0].nodeValue
                if len(cfgElement.getElementsByTagName('name')[0].childNodes) > 0:
                    cfg['name'] = cfgElement.getElementsByTagName('name')[0].childNodes[0].nodeValue
                if len(cfgElement.getElementsByTagName('topic')[0].childNodes) > 0:
                    cfg['topic'] = cfgElement.getElementsByTagName('topic')[0].childNodes[0].nodeValue
                if len(cfgElement.getElementsByTagName('proxyname')[0].childNodes) > 0:
                    cfg['proxyName'] = cfgElement.getElementsByTagName('proxyname')[0].childNodes[0].nodeValue
                if len(cfgElement.getElementsByTagName('ip')[0].childNodes) > 0:
                    cfg['ip'] = cfgElement.getElementsByTagName('ip')[0].childNodes[0].nodeValue
                if len(cfgElement.getElementsByTagName('port')[0].childNodes) > 0:
                    cfg['port'] = cfgElement.getElementsByTagName('port')[0].childNodes[0].nodeValue
                if len(cfgElement.getElementsByTagName('interface')[0].childNodes) > 0:
                    cfg['interface'] = cfgElement.getElementsByTagName('interface')[0].childNodes[0].nodeValue

            if cfg is not None:
                configs.append(cfg)

        libraries = []

        # parse libraries
        libraryElements = doc.getElementsByTagName('VisualStates')[0].getElementsByTagName('libraries')
        if len(libraryElements) > 0:
            libraryElements = libraryElements[0].getElementsByTagName('library')
            for libElement in libraryElements:
                libraries.append(libElement.childNodes[0].nodeValue)

        return (rootState, configs, libraries)

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






