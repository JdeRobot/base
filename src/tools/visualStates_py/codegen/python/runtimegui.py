from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPainter, QPixmap
from PyQt5.QtWidgets import QMainWindow, QDockWidget, QTreeView, QGraphicsView, \
    QWidget, QLabel, QVBoxLayout, QPushButton, QGraphicsItem, \
    QGraphicsScene


from gui.treemodel import TreeModel
from gui.state import State
from gui.transition import Transition

from threading import Thread
import time
import sysv_ipc

class RunTimeGui(QMainWindow):

    activeStateChanged = pyqtSignal(int)
    runningStateChanged = pyqtSignal(int)
    loadFromRoot = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__()

        self.setWindowTitle("VisualStates RunTime GUI")

        # # root state
        self.rootState = None
        #self.activeState = self.rootState

        # create status bar
        self.statusBar()

        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

        self.states = {}
        self.transitions = {}

        self.activeState = None

        self.activeStateChanged.connect(self.activeStateChangedHandle)
        self.runningStateChanged.connect(self.runningStateChangedHandle)
        self.loadFromRoot.connect(self.loadFromRootHandle)

        self.memory = None
        self.ipcThread = None

    def createTreeView(self):
        dockWidget = QDockWidget()
        dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea)
        dockWidget.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dockWidget.setTitleBarWidget(QWidget())
        self.treeView = QTreeView()
        self.treeView.clicked.connect(self.treeItemClicked)
        self.treeModel = TreeModel()
        self.treeView.setModel(self.treeModel)

        self.logo = QLabel()
        logoPixmap = QPixmap('/usr/local/share/jderobot/resources/jderobot.png')
        self.logo.setPixmap(logoPixmap)

        self.upButton = QPushButton()
        self.upButton.setText('Up')
        self.upButton.clicked.connect(self.upButtonClicked)

        leftContainer = QWidget()
        leftLayout = QVBoxLayout()
        leftLayout.addWidget(self.treeView)
        leftLayout.addWidget(self.upButton)
        leftLayout.addWidget(self.logo)
        leftContainer.setLayout(leftLayout)

        dockWidget.setWidget(leftContainer)
        self.addDockWidget(Qt.LeftDockWidgetArea, dockWidget)

    def createStateCanvas(self):
        self.stateCanvas = QGraphicsView()
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(0, 0, 2000, 2000)
        # self.automataScene.activeStateChanged.connect(self.activeStateChanged)
        # self.automataScene.stateInserted.connect(self.stateInserted)
        # self.automataScene.stateRemoved.connect(self.stateRemoved)
        # self.automataScene.transitionInserted.connect(self.transitionInserted)
        # self.automataScene.stateNameChangedSignal.connect(self.stateNameChanged)

        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setScene(self.scene)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)


    def addState(self, id, name, initial, x, y, parentId):
        if parentId is not None:
            self.states[id] = State(id, name, initial, self.states[parentId])
            self.states[parentId].addChild(self.states[id])
            parentItem = self.treeModel.getByDataId(parentId)
            print('parent:' + str(parentItem))
            # self.treeModel.insertState(self.states[id].getGraphicsItem(), Qt.white, parentItem)
        else:
            self.states[id] = State(id, name, initial, None)
            # self.treeModel.insertState(self.states[id].getGraphicsItem(), Qt.white)
        if id == 0:
            self.rootState = self.states[id]

        self.states[id].setPos(x, y)

    def addTransition(self, id, name, originId, destinationId, x, y):
        self.transitions[id] = Transition(id, name, self.states[originId], self.states[destinationId])
        self.transitions[id].setPos(x, y)

    def emitRunningStateById(self, id):
        print('emit running state:' + str(id))
        self.runningStateChanged.emit(id)

    def runningStateChangedHandle(self, id):
        # if self.waitForActiveState:
        #     return

        print('running state:' + str(id))
        if id not in self.states:
            return

        runningState = self.states[id]

        parentId = None
        if runningState.parent is not None:
            for child in runningState.parent.getChildren():
                child.setRunning(False)
                # if self.activeState == child:
                #     for grandChild in self.activeState.getChildren():
                #         grandChild.setRunning(False)

            runningState.setRunning(True)
            parentId = runningState.parent.id

        self.treeModel.setAllBackgroundByParentId(Qt.white, parentId)
        self.treeModel.setBackgroundById(runningState.id, Qt.green)

    def emitActiveStateById(self, id):
        self.activeStateChanged.emit(id)

    def activeStateChangedHandle(self, id):
        # self.waitForActiveState = True

        if self.activeState is not None:
            for child in self.activeState.getChildren():
                child.resetGraphicsItem()
                for tran in child.getOriginTransitions():
                    tran.resetGraphicsItem()

        self.activeState = self.states[id]
        print('set active state:' + str(id))
        self.scene.clear()
        for childState in self.activeState.getChildren():
            print('add child to scene:' + str(childState.id))
            # childState.resetGraphicsItem()
            qitem = childState.getGraphicsItem()
            qitem.setAcceptHoverEvents(False)
            qitem.setFlag(QGraphicsItem.ItemIsMovable, False)
            qitem.doubleClicked.connect(self.stateDoubleClicked)
            self.setAcceptDrops(False)
            self.scene.addItem(qitem)
            for tran in childState.getOriginTransitions():
                print('add transition:' + str(tran.id))
                # tran.resetGraphicsItem()
                qitem = tran.getGraphicsItem()
                qitem.disableInteraction()
                self.scene.addItem(qitem)
        # self.waitForActiveState = False

    def emitLoadFromRoot(self):
        self.loadFromRoot.emit(0)

    def loadFromRootHandle(self, id):
        self.treeModel.loadFromRoot(self.states[id])

    def stateDoubleClicked(self, stateItem):
        # print('emit active state by id:' + str(stateItem.stateData.id))
        if len(stateItem.stateData.getChildren()) > 0:
            self.emitActiveStateById(stateItem.stateData.id)


    # def stateInserted(self, state):
    #     if self.activeState != self.rootState:
    #         parent = self.treeModel.getByDataId(self.activeState.id)
    #         self.treeModel.insertState(state, QColor(Qt.white), parent)
    #     else:
    #         self.treeModel.insertState(state, QColor(Qt.white))
    #
    # def stateRemoved(self, state):
    #     if self.activeState != self.rootState:
    #         self.treeModel.removeState(state.stateData, self.activeState)
    #     else:
    #         self.treeModel.removeState(state.stateData)
    #
    # def transitionInserted(self, tran):
    #     print('transition inserted:' + tran.transitionData.name)
    #
    # def stateNameChanged(self, state):
    #     dataItem = self.treeModel.getByDataId(state.stateData.id)
    #     if dataItem != None:
    #         dataItem.name = state.stateData.name
    #         self.treeModel.layoutChanged.emit()

    # def activeStateChanged(self):
    #     if self.automataScene.activeState != self.activeState:
    #         print('visual states active state changed:' + self.automataScene.activeState.name)
    #         self.activeState = self.automataScene.activeState
    #         if self.activeState == self.rootState:
    #             self.treeView.selectionModel().clearSelection()
    #         else:
    #             self.treeView.setCurrentIndex(self.treeModel.indexOf(self.treeModel.getByDataId(self.activeState.id)))

    def upButtonClicked(self):
        if self.activeState is not None:
            if self.activeState.parent is not None:
                self.emitActiveStateById(self.activeState.parent.id)
                # print('parent name:' + self.activeState.parent.name)
                # self.automataScene.setActiveState(self.activeState.parent)
                # if self.activeState.parent == self.rootState:
                #     self.treeView.selectionModel().clearSelection()
                #     print('clear selection')


    def getStateById(self,state, id):
        if state.id == id:
            return state
        else:
            result = None
            for child in state.getChildren():
                result = self.getStateById(child, id)
                if result is not None:
                    return result
            return result

    def treeItemClicked(self, index):
        print('clicked item.id:' + str(index.internalPointer().id))
        # state = self.getStateById(self.rootState, index.internalPointer().id)
        # if state is not None:
        #     # set the active state as the loaded state
        #     self.automataScene.setActiveState(state)
        #     if state == self.rootState:
        #         self.treeView.selectionModel().clearSelection()
        #     else:
        #         self.treeView.setCurrentIndex(self.treeModel.indexOf(self.treeModel.getByDataId(self.activeState.id)))

    # def timeStepDurationChanged(self, duration):
    #     if self.activeState is not None:
    #         self.activeState.setTimeStep(duration)
    #
    # def variablesChanged(self, variables):
    #     if self.activeState is not None:
    #         self.activeState.setVariables(variables)
    #
    # def functionsChanged(self, functions):
    #     if self.activeState is not None:
    #         self.activeState.setFunctions(functions)
    #
    # def librariesChanged(self, libraries):
    #     self.libraries = libraries
    #
    # def configsChanged(self, configs):
    #     self.configs = configs

    def getStateList(self, state, stateList):
        if len(state.getChildren()) > 0:
            stateList.append(state)

        for s in state.getChildren():
            self.getStateList(s, stateList)

    def loopIPC(self):
        while True:
            msg = self.getIPCMessage()
            if msg is not None:
                print('msg received:' + msg)
                methodName = msg.split(' ')[0]
                id = int(msg.split(' ')[1])
                if methodName == 'emitRunningStateById':
                    self.emitRunningStateById(id)
                else:
                    print('unknown method name')

            time.sleep(1.0/1000)

    def activateIPC(self):
        # fp0 = open('/tmp/visualstates', 'w')
        # for i in range(1024):
        #     fp0.write('0')
        # fp0.close()

        # fp = open('/tmp/visualstates', 'r+b')
        # self.mm = mmap.mmap(fp.fileno(), 1024)
        # Create shared memory object
        self.memory = sysv_ipc.SharedMemory(123456, sysv_ipc.IPC_CREAT)
        self.ipcThread = Thread(target=self.loopIPC)
        self.ipcThread.start()

    def getIPCMessage(self):
        if self.memory is not None:
            data = self.memory.read().decode()
            if data[0] != '0':
                self.memory.write('0'.encode())
                i = data.find('\0')
                if i != -1:
                    data = data[:i]
                return data

        return None





