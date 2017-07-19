
from xml.dom import minidom

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



