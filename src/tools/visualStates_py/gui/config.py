
JDEROBOTCOMM = 0
ROS = 1

class Config():
    def __init__(self):
        self.type = JDEROBOTCOMM


class RosConfig(Config):
    def __init__(self):
        self.type = ROS
        self.topics = []
        self.buildDependencies = []
        self.runDependencies = []

    def getTopics(self):
        return self.topics

    def addTopic(self, id, name, type):
        topic = {}
        topic['id'] = id
        topic['name'] = name
        topic['type'] = type
        self.topics.append(topic)

    def removeTopic(self, id):
        topicToDelete = None
        for t in self.topics:
            if t['id'] == id:
                topicToDelete = t
                break
        self.topics.remove(topicToDelete)


    def setBuildDependencies(self, dependencies):
        pass

    def getBuildDependencies(self):
        pass

    def setRunDependencies(self, dependencies):
        pass

    def getRunDependencies(self):
        pass

    def createNode(self, doc):
        cfgElement = doc.createElement('config')
        cfgElement.setAttribute('type', str(self.type))

        bDependencies = doc.createElement('buildDependencies')
        for bDepend in self.buildDependencies:
            dependElement = doc.createElement('dependency')
            dependElement.appendChild(doc.createTextNode(bDepend))
            bDependencies.appendChild(dependElement)
        cfgElement.appendChild(bDependencies)

        rDependencies = doc.createElement('runDependencies')
        for rDepend in self.runDependencies:
            dElement = doc.createElement('dependency')
            dElement.appendChild(doc.createTextNode(rDepend))
            rDependencies.appendChild(dElement)
        cfgElement.appendChild(rDependencies)

        tElements = doc.createElement('topics')
        for t in self.topics:
            tElement = doc.createElement('topic')
            tElement.setAttribute('id', str(t['id']))
            nameElement = doc.createElement('name')
            nameElement.appendChild(doc.createTextNode(t['name']))
            tElement.appendChild(nameElement)
            typeElement = doc.createElement('type')
            typeElement.appendChild(doc.createTextNode(t['type']))
            tElement.appendChild(typeElement)
            tElements.appendChild(tElement)
        cfgElement.appendChild(tElements)

        return cfgElement

    def loadNode(self, node):
        self.type = node.getAttribute('type')

        self.buildDependencies = []
        bDependencies = node.getElementsByTagName('buildDependencies')[0]
        for bDependency in bDependencies.getElementsByTagName('dependency'):
            self.buildDependencies.append(bDependency.childNodes[0].nodeValue)
            print('bdepend:' + bDependency.childNodes[0].nodeValue)

        self.runDependencies = []
        rDependencies = node.getElementsByTagName('runDependencies')[0]
        for rDependency in rDependencies.getElementsByTagName('dependency'):
            self.runDependencies.append(rDependency.childNodes[0].nodeValue)
            print('rdepend:' + rDependency.childNodes[0].nodeValue)

        self.topics = []
        tElements = node.getElementsByTagName('topics')[0]
        for t in self.tElements.getElementsByTagName('topic'):
            topic = {}
            topic['id'] = int(t.getAttribute('id'))
            topic['name'] = t.getElementsByTagName('name')[0].childNodes[0].nodeValue
            topic['type'] = t.getElementsByTagName('type')[0].childNodes[0].nodeValue
            self.topics.append(topic)



class JdeRobotConfig(Config):
    def __init__(self):
        self.type = JDEROBOTCOMM
        self.interfaces = []

    def getInterfaces(self):
        return self.interfaces

    def removeInterface(self, id):
        deleteItem = None
        for inter in self.interfaces:
            if inter['id'] == id:
                deleteItem = inter
                break
        if deleteItem is not None:
            self.interfaces.remove(deleteItem)


    def addInterface(self, interface):
        self.interfaces.append(interface)

    def createNode(self, doc):
        cfgElement = doc.createElement('config')
        cfgElement.setAttribute('type', str(self.type))

        interfacesElement = doc.createElement('interfaces')
        for inter in self.interfaces:
            interElement = doc.createElement('interface')

            sTypeElement = doc.createElement('serverType')
            sTypeElement.appendChild(doc.createTextNode(inter['serverType']))
            interElement.appendChild(sTypeElement)

            nameElement = doc.createElement('name')
            nameElement.appendChild(doc.createTextNode(inter['name']))
            interElement.appendChild(nameElement)

            topicElement = doc.createElement('topic')
            topicElement.appendChild(doc.createTextNode(inter['topic']))
            interElement.appendChild(topicElement)

            proxyElement = doc.createElement('proxyName')
            proxyElement.appendChild(doc.createTextNode(inter['proxyName']))
            interElement.appendChild(proxyElement)

            ipElement = doc.createElement('ip')
            ipElement.appendChild(doc.createTextNode(inter['ip']))
            interElement.appendChild(ipElement)

            portElement = doc.createElement('port')
            portElement.appendChild(doc.createTextNode(inter['port']))
            interElement.appendChild(portElement)

            iElement = doc.createElement('interfaceName')
            iElement.appendChild(doc.createTextNode(inter['interface']))
            interElement.appendChild(iElement)

            interfacesElement.appendChild(interElement)

        return cfgElement

    def loadNode(self, node):
        self.type = node.getAttribute('type')
        interfacesElement = node.getElementsByTagName('interfaces')[0]
        self.interfaces = []
        for interElement in interfacesElement.getElementsByTagName('interface'):
            interface = {}
            sTypeElement = interElement.getElementsByTagName('serverType')[0]
            interface['serverType'] = sTypeElement.childNodes[0].nodeValue

            nameElement = interElement.getElementsByTagName('name')[0]
            interface['name'] = nameElement.childNodes[0].nodeValue

            topicElement = interElement.getElementsByTagName('topic')[0]
            interface['topic'] = topicElement.childNodes[0].nodeValue

            proxyElement = interElement.getElementsByTagName('proxyName')[0]
            interface['proxyName'] = proxyElement.childNodes[0].nodeValue

            ipElement = interElement.getElementsByTagName('ip')[0]
            interface['ip'] = ipElement.childNodes[0].nodeValue

            portElement = interElement.getElementsByTagName('port')[0]
            interface['port'] = portElement.childNodes[0].nodeValue

            iElement = interElement.getElementsByTagName('interfaceName')[0]
            interface['interface'] = iElement.childNodes[0].nodeValue

            self.interfaces.append(interface)


