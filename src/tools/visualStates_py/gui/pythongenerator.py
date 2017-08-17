from gui.generator import Generator
from gui.transitiontype import TransitionType
import os

class PythonGenerator(Generator):
    def __init__(self, libraries, configs, interfaceHeaders, states):
        self.libraries = libraries
        self.configs = configs
        self.interfaceHeaders = interfaceHeaders
        self.states = states


    def generate(self, projectPath, projectName):
        stringList = []
        self.generateHeaders(stringList)
        self.generateAutomataClass(stringList)
        self.generateMain(stringList)
        sourceCode = ''.join(stringList)
        fp = open(projectPath + os.sep + projectName + '.py')
        fp.write(sourceCode)
        fp.close()

        stringList = []
        self.generateCfg(stringList)
        fp = open(projectPath + os.sep + projectName + '.cfg')
        fp.write(''.join(stringList))
        fp.close()

        os.system('chmod +x ' + projectPath + os.sep + projectName + '.py')


    def generateHaders(self, headerStr):
        mystr = '''
        #!/usr/bin/python
        # -*- coding: utf-8 -*-
        
        import Ice
        import easyiceconfig as EasyIce
        import sys, signal
        sys.path.append('/opt/jderobot/share/jderobot/python/visualStates_py)
        import traceback, threading, time
        from automatagui import Automatagui, QtGui, GuiSubautomata
        '''
        headerStr.append(mystr)
        for lib in self.libraries:
            headerStr.append('import ')
            headerStr.append(lib)
            headerStr.append('\n')
        headerStr.append('\n')

        for cfg in self.configs:
            headerStr.append('from jderobot import')
            headerStr.append(cfg['interface'])
            headerStr.append('Prx\n')

        headerStr.append('\n')

        return headerStr

    def generateAutomataClass(self, automataStr):
        automataStr.append('class Automata():\n\n')
        self.generateAutomataInit(automataStr)
        self.generateUserFunctions(automataStr)
        self.generateStartThreads(automataStr)
        self.generateCreateGui(automataStr)
        self.generateShutDown(automataStr)
        self.generateRunGui(automataStr)
        self.generateAutomata(automataStr)
        self.generateConnectProxies(automataStr)
        self.generateDestroyIc(automataStr)
        self.generateStart(automataStr)
        self.generateJoin(automataStr)
        self.generateReadArgs(automataStr)


    def generateAutomataInit(self, automataStr):
        mystr = '''
        def __init__(self):
            self.lock = threading.Lock()
            self.displayGui = False
        '''
        automataStr.append(mystr)

        for state in self.states:
            automataStr.append('\t\tself.StatesSub')
            automataStr.append(str(state.id))
            automataStr.append(' = [\n')

            for childState in state.getChildren():
                automataStr.append('\t\t\t')
                automataStr.append('"')
                automataStr.append(childState.name)
                automataStr.append('",\n')

                if childState.parent is not None:
                    automataStr.append('\t\t\t"')
                    automataStr.append(childState.name)
                    automataStr.append('_ghost')
                    automataStr.append('",')

            automataStr.append('\t\t]\n\n')

        for state in self.states:
            initialState = None
            for childState in state.getChildren():
                if childState.initial():
                    initialState = childState
                    break

            stateName = initialState.name
            if state.parent is not None:
                stateName += '_ghost'

            automataStr.append('\t\t')
            automataStr.append('self.sub')
            automataStr.append(str(state.id))
            automataStr.append(' = "')
            automataStr.append(stateName)
            automataStr.append('"\n')
            automataStr.append('\t\t')
            automataStr.append('self.run')
            automataStr.append(str(state.id))
            automataStr.append(' = True\n')

        automataStr.append('\n')

        return automataStr

    def generateStartThreads(self, threadStr):
        threadStr.append('def startThreads(self):\n')

        for state in self.states:
            threadStr.append('\tself.t')
            threadStr.append(str(state.id))
            threadStr.append(' = threading.Thread(target=self.subautomata')
            threadStr.append(str(state.id))
            threadStr.append('\n')
            threadStr.append('\tself.t')
            threadStr.append(str(state.id))
            threadStr.append('.start()\n')

        threadStr.append('\n')

    def generateCreateGui(self, guiStr):
        mystr = '''
        def createAutomata(self):
            guiSubautomataList = []
        '''

        for state in self.states:
            guiStr.append('# Creating subAutomata')
            guiStr.append(str(state.id))
            guiStr.append('\n')
            guiStr.append('\tguiSubautomata')
            guiStr.append(str(state.id))
            guiStr.append(' = GuiSubautomata(')
            guiStr.append(str(state.id))
            guiStr.append(', ')
            guiStr.append(str(state.parent.id))
            guiStr.append(', self.automataGui)\n\n')

            for childState in state.getChildren():
                guiStr.append('\tguiSubautomata')
                guiStr.append(str(state.id))
                guiStr.append('.newGuiNode(')
                guiStr.append(str(childState.id))
                guiStr.append(', ')
                guiStr.append(str(childState.parent.id))
                guiStr.append(', ')
                guiStr.append(str(childState.x))
                guiStr.append(', ')
                guiStr.append(str(childState.y))
                guiStr.append(', ')
                guiStr.append(str(childState.initial))
                guiStr.append(', "')
                guiStr.append(childState.name)
                guiStr.append('")\n')

            for tran in state.getOriginTransitions():
                guiStr.append('\tguiSubautomata')
                guiStr.append(str(state.id))
                guiStr.append('.newGuiTransition((')
                guiStr.append(str(tran.origin.x))
                guiStr.append(', ')
                guiStr.append(str(tran.origin.y))
                guiStr.append('), (')
                guiStr.append(str(tran.destination.x))
                guiStr.append(', ')
                guiStr.append(str(tran.destination.y))
                guiStr.append('), (')
                guiStr.append(str(tran.x))
                guiStr.append(', ')
                guiStr.append(str(tran.y))
                guiStr.append('), ')
                guiStr.append(str(tran.id))
                guiStr.append(', ')
                guiStr.append(str(tran.origin.id))
                guiStr.append(', ')
                guiStr.append(str(tran.destination.id))
                guiStr.append(')\n')

            guiStr.append('\tguiSubautomataList.append(guiSubautomata')
            guiStr.append(str(state.id))
            guiStr.append(')\n\n')
        guiStr.append('\treturn guiSubautomataList')

        return guiStr


    def generateShutDown(self, shutdownStr):
        shutdownStr.append('\tdef shutDown(self):\n')

        for state in self.states:
            shutdownStr.append('\t\tself.run')
            shutdownStr.append(str(state.id))
            shutdownStr.append(' = False\n')

        shutdownStr.append('\n')

        return shutdownStr


    def generateRunGui(self, runStr):
        mystr = '''
        def runGui(self):
            app = QtGui.QApplication(sys.argv)
            self.automataGui = AutomataGui()
            self.automataGui.setAutomata(self.createAutomata())
            self.automataGui.loadAutomata()
            self.startThreads()
            self.automataGui.show()
            app.exec_()
            
        '''
        runStr.append(mystr)

        return runStr


    def generateAutomata(self, automataStr):
        for state in self.states:
            automataStr.append('\tdef subautomata')
            automataStr.append(str(state.id))
            automataStr.append('(self):\n')

            automataStr.append('\t\tself.run')
            automataStr.append(str(state.id))
            automataStr.append(' = True\n')
            automataStr.append('\t\tcycle = ')
            automataStr.append(str(state.getTimeStep()))
            automataStr.append('\n')
            automataStr.append('\t\tt_activated = False\n')
            automataStr.append('\t\tt_fin = 0\n\n')

            nameTimeMap = {}

            if state.parent is not None:
                for tran in state.getOriginTransitions():
                    if tran.getType() == TransitionType.TEMPORAL:
                        automataStr.append('\t\tt_')
                        automataStr.append(tran.origin.name)
                        automataStr.append('_max = ')
                        automataStr.append(str(tran.getTemporalTime() / 1000.0))
                        automataStr.append('\n')
                        nameTimeMap[tran.origin.name] = tran.getTemporalTime() / 1000.0

                automataStr.append('\n')

            for line in state.getVariables().split('\n'):
                automataStr.append('\t\t')
                automataStr.append(line)
                automataStr.append('\n')
            automataStr.append('\n')

            automataStr.append('\t\twhile(self.run')
            automataStr.append(str(state.id))
            automataStr.append('):\n')
            automataStr.append('\t\t\ttotala = time.time() * 1000000\n\n')

            addTab = None
            if state.parent is not None:
                automataStr.append('\t\t\tif(self.sub')
                automataStr.append(str(state.parent.id))
                automataStr.append(' == "')
                automataStr.append(state.parent.name)
                automataStr.append('"')
                automataStr.append(');\n')
                automataStr.append('\t\t\t\t')
                automataStr.append('if (')
                for childState in state.getChildren():
                    automataStr.append('(self.sub')
                    automataStr.append(str(state.id))
                    automataStr.append(' == "')
                    automataStr.append(state.name)
                    automataStr.append('_ghost")')
                    if childState is not state.getChildren()[len(state.getChildren())-1]:
                        automataStr.append(' or ')

                automataStr.append('):\n')

                automataStr.append('\t\t\t\t\tghostStateIndex = self.StatesSub')
                automataStr.append(str(state.id))
                automataStr.append('.index(self.sub')
                automataStr.append(str(state.id))
                automataStr.append(')\n')
                automataStr.append('\t\t\t\t\tself.sub')
                automataStr.append(str(state.id))
                automataStr.append(' = self.StatesSub')
                automataStr.append(str(state.id))
                automataStr.append('[ghostStateIndex-1]\n')
                automataStr.append('\t\t\t\t\tt_ini = time.time()\n\n')

                addTab = 1
            else:
                addTab = 0

            tabStr = '\t\t\t'
            if addTab == 1:
                tabStr += '\t'

            automataStr.append(tabStr)
            automataStr.append('#Evaluation \n')

            firstState = True

            for childState in state.getChildren():
                firstTransition = True
                if firstState:
                    automataStr.append(tabStr)
                    automataStr.append('if (self.sub')
                    automataStr.append(str(state.id))
                    automataStr.append(' == "')
                    automataStr.append(childState.name)
                    automataStr.append('"):\n')
                    firstState = False
                else:
                    automataStr.append(tabStr)
                    automataStr.append('elif (self.sub')
                    automataStr.append(str(state.id))
                    automataStr.append(' == "')
                    automataStr.append(childState.name)
                    automataStr.append('"):\n')

                ifHeaderUsed = False
                for tran in state.getOriginTransitions():
                    if tran.origin.id == childState.id:
                        if not ifHeaderUsed:
                            automataStr.append(tabStr+'\t')
                            ifHeaderUsed = True

                        if tran.getType() == TransitionType.CONDITIONAL:
                            automataStr.append(tabStr+'\t\t')
                            if firstTransition:
                                automataStr.append('if (')
                                automataStr.append(tran.getCondition())
                                automataStr.append('):\n')
                                firstTransition = False
                            else:
                                automataStr.append('elif(')
                                automataStr.append(tran.getCondition())
                                automataStr.append('):\n')

                            automataStr.append('\t\t\t\t\t\tself.sub')
                            automataStr.append(str(state.id))
                            automataStr.append(' = "')
                            automataStr.append(tran.destination.name)
                            automataStr.append('"\n')

                            for codeLine in tran.getCode().split('\n'):
                                automataStr.append('\t\t\t\t\t\t')
                                automataStr.append(codeLine)
                                automataStr.append('\n')

                            automataStr.append('\t\t\t\t\t\tif self.displayGui:\n')
                            automataStr.append('\t\t\t\t\t\t\tself.automataGui.notifySetNodeAsActive("\n')
                            automataStr.append(str(tran.destination.id))
                            automataStr.append('")\n')
                        else:
                            automataStr.append(tabStr+'\t')
                            automataStr.append('if (not t_activated):\n')
                            automataStr.append(tabStr+'\t\t')
                            automataStr.append('t_ini = time.time()\n')
                            automataStr.append(tabStr+'\t\t')
                            automataStr.append('t_activated = True\n')
                            automataStr.append(tabStr+'\t')
                            automataStr.append('else:\n')
                            automataStr.append(tabStr+'\t\t')
                            automataStr.append('t_fin = time.time()\n')
                            automataStr.append(tabStr+'\t\t')
                            automataStr.append('secs = t_fin - t_ini\n')
                            automataStr.append(tabStr+'\t\t')
                            if state.parent is None:
                                automataStr.append('if (secs > ')
                                automataStr.append(str(tran.getTemporalTime() / 1000.0))
                                automataStr.append('):\n')
                            else:
                                automataStr.append('if (secs > t_')
                                automataStr.append(childState.name)
                                automataStr.append('_max):\n')

                            automataStr.append(tabStr+'\t\t\t')
                            automataStr.append('self.sub')
                            automataStr.append(str(state.id))
                            automataStr.append(' = "')
                            automataStr.append(tran.destination.name)
                            automataStr.append('"\n')
                            automataStr.append(tabStr+'\t\t\t')
                            automataStr.append('t_activated = False\n')

                            for codeLine in tran.getCode().split('\n'):
                                automataStr.append(tabStr+'\t\t\t')
                                automataStr.append(codeLine)
                                automataStr.append('\n')

                            automataStr.append(tabStr+'\t\t\t')
                            automataStr.append('if self.displayGui:\n')
                            automataStr.append(tabStr+'\t\t\t\t')
                            automataStr.append('self.automataGui.notifySetNodeAsActive("')
                            automataStr.append(tran.destination.name)
                            automataStr.append('")\n')

                            if state.parent is not None:
                                automataStr.append(tabStr+'\t\t\t')
                                automataStr.append('t_')
                                automataStr.append(tran.origin.name)
                                automataStr.append('_max = ')
                                automataStr.append(nameTimeMap[tran.origin.name])
                                automataStr.append('\n')

                        automataStr.append('\n')
            automataStr.append('\n')

            automataStr.append(tabStr)
            automataStr.append('# Actuation\n')

            firstState = True
            for childState in state.getChildren():
                if len(childState.getCode()) > 0:
                    automataStr.append(tabStr)
                    if firstState:
                        automataStr.append('if (self.sub')
                        automataStr.append(str(state.id))
                        automataStr.append(' == "')
                        automataStr.append(childState.name)
                        automataStr.append('"):\n')
                        firstState = False
                    else:
                        automataStr.append('elif (self.sub')
                        automataStr.append(str(state.id))
                        automataStr.append(' == "')
                        automataStr.append(childState.name)
                        automataStr.append('"):\n')

                for codeLine in childState.getCode().split('\n'):
                    automataStr.append(tabStr+'\t')
                    automataStr.append(codeLine)
                    automataStr.append('\n')

            if state.parent is not None:
                firstState = True

                for childState in state.getChildren():
                    if firstState:
                        automataStr.append('\t\t\telse:\n')
                        automataStr.append('\t\t\t\tif (self.sub')
                        automataStr.append(str(state.id))
                        automataStr.append(' == "')
                        automataStr.append(childState.name)
                        automataStr.append('"):\n')
                        firstState = False
                    else:
                        automataStr.append('\t\t\t\t')
                        automataStr.append('elif (self.sub')
                        automataStr.append(str(state.id))
                        automataStr.append(' == "')
                        automataStr.append(childState.name)
                        automataStr.append('"):\n')
                    if childState.name in nameTimeMap:
                        automataStr.append('\t\t\t\t\t')
                        automataStr.append('ghostStateIndex = self.StatesSub')
                        automataStr.append(str(state.id))
                        automataStr.append('.index(self.sub')
                        automataStr.append(str(state.id))
                        automataStr.append(') + 1\n')
                        automataStr.append('\t\t\t\t\tself.sub')
                        automataStr.append(str(state.id))
                        automataStr.append(' = self.StatesSub')
                        automataStr.append(str(state.id))
                        automataStr.append('[ghostStateIndex]\n')

            automataStr.append('\n')

            automataStr.append('\t\t\ttotalb = time.time() * 1000000\n')
            automataStr.append('\t\t\tmsecs = (totalb - totala) / 1000\n')
            automataStr.append('\t\t\tif (msecs < 0 or msecs > cycle):\n')
            automataStr.append('\t\t\t\tmsecs = cycle\n')
            automataStr.append('\t\t\telse:\n')
            automataStr.append('\t\t\t\tmsecs = cycle - msecs\n\n')
            automataStr.append('\t\t\ttime.sleep(msecs / 1000)\n')
            automataStr.append('\t\t\tif (msecs < 33):\n')
            automataStr.append('\t\t\t\ttime.sleep(33 / 1000)\n\n\n')

        return automataStr

    def generateConnectProxies(self, proxyStr):
        proxyStr.append('def connectToProxies(self):\n')
        proxyStr.append('\tself.ic = EasyIce.initialize(sys.argv)\n\n')

        for cfg in self.configs:
            proxyStr.append('\t#Contact to ')
            proxyStr.append(cfg['name'])
            proxyStr.append('\n')

            proxyStr.append(cfg['name'])
            proxyStr.append(' = self.ic.propertyToProxy(automata.')
            proxyStr.append(cfg['name'])
            proxyStr.append('.Proxy)\n')

            proxyStr.append('\tif (not ')
            proxyStr.append(cfg['name'])
            proxyStr.append('):\n')
            proxyStr.append('\t\traise Exception("could not create proxy with')
            proxyStr.append(cfg['name'])
            proxyStr.append('")\n')

            proxyStr.append('\tself.')
            proxyStr.append(cfg['name'])
            proxyStr.append(' = ')
            proxyStr.append(cfg['interface'])
            proxyStr.append('.Prx.checkedCast(')
            proxyStr.append(cfg['name'])
            proxyStr.append(')\n')

            proxyStr.append('\tif (not self.')
            proxyStr.append(cfg['name'])
            proxyStr.append('):\n')
            proxyStr.append('\t\traise Exception("invalid proxy automata.')
            proxyStr.append(cfg['name'])
            proxyStr.append('.Proxy")\n')
            proxyStr.append('\tprint("')
            proxyStr.append(cfg['name'])
            proxyStr.append(' connected")\n\n')

        return proxyStr

    def generateDestroyIc(self, icStr):
        mystr = '''
    def destroyIc(self):
        if (self.ic):
            self.ic.destroy()
    

'''
        icStr.append(mystr)
        return icStr


    def generateStart(self, startStr):
        mystr = '''
    def start(self):
        if self.displayGui:
            self.guiThread = threading.Thread(target=self.runGui)
            self.guiThread.start()
        else:
            self.startThreads()

'''
        startStr.append(mystr)
        return startStr

    def generateJoin(self, joinStr):
        joinStr.append('\tdef join(self):\n')
        joinStr.append('\t\tif self.displayGui:\n')
        joinStr.append('\t\t\tself.guiThread.join()\n')

        for state in self.states:
            joinStr.append('\t\tself.t')
            joinStr.append(str(state.id))
            joinStr.append('.join()\n')

        joinStr.append('\n\n')

        return joinStr


    def generateReadArgs(self, argsStr):
        mystr = '''
    def readArgs(self):
        for arg in sys.argv:
            splitedArg = arg.split('=')
            if splitedArg[0] == "--displaygui":
                if splitedArg[1] == "True" or splitedArg[1] == "true":
                    self.displayGui = True
                    print("runtime gui enabled")
                else:
                    self.displayGui = False
                    print("runtime gui disabled")
                    
'''
        argsStr.append(mystr)
        return argsStr

    def generateMain(self, mainStr):
        mystr = '''
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    automata = Automata()
    try:
        automata.connectToProxies()
        automata.readArgs()
        automata.start()
        automata.join()
        
        sys.exit(0)
    except:
        traceback.print_exc()
        automata.destroyIc()
        sys.exit(-1)
        
'''
        mainStr.append(mystr)































