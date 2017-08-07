from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPixmap
from PyQt5.QtWidgets import QMainWindow, QAction, QDockWidget, QTreeView, QGraphicsView, QWidget, QFileDialog, QLabel, QVBoxLayout, QPushButton

from gui.automatascene import AutomataScene, OpType
from gui.filemanager import FileManager
from gui.treemodel import TreeModel
from .guistate import StateGraphicsItem
from .state import State


class VisualStates(QMainWindow):
    def __init__(self, parent=None):
        super().__init__()

        self.setWindowTitle("VisualStates")

        # root state
        self.rootState = State(0, "root", True)
        self.activeState = self.rootState

        # create status bar
        self.statusBar()

        self.createMenu()
        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

        self.fileManager = FileManager()

    def createMenu(self):
        # create actions
        # archieve menu
        newAction = QAction('&New', self)
        newAction.setShortcut('Ctrl+N')
        newAction.setStatusTip('Create New Visual States')
        newAction.triggered.connect(self.newAction)

        openAction = QAction('&Open', self)
        openAction.setShortcut('Ctrl+O')
        openAction.setStatusTip('Open Visual States')
        openAction.triggered.connect(self.openAction)

        saveAction = QAction('&Save', self)
        saveAction.setShortcut('Ctrl+S')
        saveAction.setStatusTip('Save Visual States')
        saveAction.triggered.connect(self.saveAction)

        saveAsAction = QAction('&Save As', self)
        saveAsAction.setShortcut('Ctrl+S')
        saveAsAction.setStatusTip('Save Visual States as New One')
        saveAsAction.triggered.connect(self.saveAsAction)

        quitAction = QAction('&Quit', self)
        quitAction.setShortcut('Ctrl+Q')
        quitAction.setStatusTip('Quit Visual States')
        quitAction.triggered.connect(self.quitAction)

        # figures menu
        stateAction = QAction('&State', self)
        # stateAction.setShortcut('Ctrl+N')
        stateAction.setStatusTip('Create a state')
        stateAction.triggered.connect(self.stateAction)

        transitionAction = QAction('&Transition', self)
        # transitionAction.setShortcut('Ctrl+T')
        transitionAction.setStatusTip('Create a transition')
        transitionAction.triggered.connect(self.transitionAction)

        # data menu
        timerAction = QAction('&Timer', self)
        timerAction.setShortcut('Ctrl+M')
        timerAction.setStatusTip('Set timing of states')
        timerAction.triggered.connect(self.timerAction)

        variablesAction = QAction('&Variables', self)
        variablesAction.setShortcut('Ctrl+V')
        variablesAction.setStatusTip('Define state variables')
        variablesAction.triggered.connect(self.variablesAction)

        functionsAction = QAction('&Functions', self)
        functionsAction.setShortcut('Ctrl+F')
        functionsAction.setStatusTip('Define functions')
        functionsAction.triggered.connect(self.functionsAction)

        # actions menu
        librariesAction = QAction('&Libraries', self)
        librariesAction.setShortcut('Ctrl+L')
        librariesAction.setStatusTip('Add additional libraries')
        librariesAction.triggered.connect(self.librariesAction)

        configFileAction = QAction('&Config File', self)
        configFileAction.setShortcut('Ctrl+C')
        configFileAction.setStatusTip('Edit configuration file')
        configFileAction.triggered.connect(self.configFileAction)

        generateCppAction = QAction('&Generate C++', self)
        generateCppAction.setShortcut('Ctrl+G')
        generateCppAction.setStatusTip('Generate C++ code')
        generateCppAction.triggered.connect(self.generateCppAction)

        compileCppAction = QAction('&Compile C++', self)
        compileCppAction.setShortcut('Ctrl+P')
        compileCppAction.setStatusTip('Compile generated C++ code')
        compileCppAction.triggered.connect(self.compileCppAction)

        generatePythonAction = QAction('&Generate Python', self)
        generatePythonAction.setShortcut('Ctrl+Y')
        generatePythonAction.setStatusTip('Generate Python code')
        generatePythonAction.triggered.connect(self.generatePythonAction)

        # help menu
        aboutAction = QAction('&About', self)
        aboutAction.setShortcut('F1')
        aboutAction.setStatusTip('Information about VisualStates')
        aboutAction.triggered.connect(self.aboutAction)

        # create main menu
        menubar = self.menuBar()
        archieveMenu = menubar.addMenu('&File')
        archieveMenu.addAction(newAction)
        archieveMenu.addAction(openAction)
        archieveMenu.addAction(saveAction)
        archieveMenu.addAction(saveAsAction)
        archieveMenu.addAction(quitAction)

        figuresMenu = menubar.addMenu('&Figures')
        figuresMenu.addAction(stateAction)
        figuresMenu.addAction(transitionAction)

        dataMenu = menubar.addMenu('&Data')
        dataMenu.addAction(timerAction)
        dataMenu.addAction(variablesAction)
        dataMenu.addAction(functionsAction)

        actionsMenu = menubar.addMenu('&Actions')
        actionsMenu.addAction(librariesAction)
        actionsMenu.addAction(configFileAction)
        actionsMenu.addAction(generateCppAction)
        actionsMenu.addAction(compileCppAction)
        actionsMenu.addAction(generatePythonAction)

        helpMenu = menubar.addMenu('&Help')
        helpMenu.addAction(aboutAction)

    def newAction(self):
        self.clearScene()
        # create new root state
        self.rootState = State(0, 'root', True)
        self.automataScene.setActiveState(self.rootState)
        self.automataScene.resetIndexes()

    def openAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Open VisualStates File")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates Files (*.xml)'])
        fileDialog.setDefaultSuffix('.xml')
        fileDialog.setAcceptMode(QFileDialog.AcceptOpen)
        if fileDialog.exec_():
            self.rootState = self.fileManager.open(fileDialog.selectedFiles()[0])
            self.treeModel.removeAll()
            self.treeModel.loadFromRoot(self.rootState)
            # set the active state as the loaded state
            self.automataScene.setActiveState(self.rootState)
        else:
            print('open is canceled')



    def saveAction(self):
        if len(self.fileManager.getFileName()) == 0:
            self.saveAsAction()
        else:
            self.fileManager.save(self.rootState)

    def saveAsAction(self):
        fileDialog = QFileDialog(self)
        fileDialog.setWindowTitle("Save As VisualStates File")
        fileDialog.setViewMode(QFileDialog.Detail)
        fileDialog.setNameFilters(['VisualStates Files (*.xml)'])
        fileDialog.setDefaultSuffix('.xml')
        fileDialog.setAcceptMode(QFileDialog.AcceptSave)
        if fileDialog.exec_():
            self.fileManager.setFileName(fileDialog.selectedFiles()[0])
            self.fileManager.save(self.rootState)
        else:
            print('file dialog canceled')


    def quitAction(self):
        print('Quit')
        self.close()

    def stateAction(self):
        self.automataScene.setOperationType(OpType.ADDSTATE)

    def transitionAction(self):
        self.automataScene.setOperationType(OpType.ADDTRANSITION)

    def timerAction(self):
        print('Timer Action')

    def variablesAction(self):
        print('Variables Action')

    def functionsAction(self):
        print('Functions Action')

    def librariesAction(self):
        print('libraries Action')

    def configFileAction(self):
        print('config file action')

    def generateCppAction(self):
        print('generate cpp action')

    def compileCppAction(self):
        print('compile cpp action')

    def generatePythonAction(self):
        print('generate python action')

    def aboutAction(self):
        print('about action')

    def createTreeView(self):
        dockWidget = QDockWidget()
        dockWidget.setAllowedAreas(Qt.LeftDockWidgetArea)
        dockWidget.setFeatures(QDockWidget.NoDockWidgetFeatures)
        dockWidget.setTitleBarWidget(QWidget())
        self.treeView = QTreeView()
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
        self.automataScene = AutomataScene()
        self.automataScene.setSceneRect(0, 0, 2000, 2000)
        self.automataScene.activeStateChanged.connect(self.activeStateChanged)
        self.automataScene.stateInserted.connect(self.stateInserted)
        self.automataScene.stateRemoved.connect(self.stateRemoved)
        self.automataScene.transitionInserted.connect(self.transitionInserted)
        self.automataScene.stateNameChangedSignal.connect(self.stateNameChanged)
        self.automataScene.setActiveState(self.rootState)

        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setScene(self.automataScene)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)
        self.stateCanvas.setAcceptDrops(True)

    def stateInserted(self, state):
        if self.activeState != self.rootState:
            parent = self.treeModel.getByDataId(self.activeState.id)
            self.treeModel.insertState(state, QColor(Qt.white), parent)
        else:
            self.treeModel.insertState(state, QColor(Qt.white))

    def stateRemoved(self, state):
        if self.activeState != self.rootState:
            self.treeModel.removeState(state.stateData, self.activeState)
        else:
            self.treeModel.removeState(state.stateData)

    def transitionInserted(self, tran):
        print('transition inserted:' + tran.transitionData.name)

    def stateNameChanged(self, state):
        dataItem = self.treeModel.getByDataId(state.stateData.id)
        if dataItem != None:
            dataItem.name = state.stateData.name
            self.treeModel.layoutChanged.emit()

    def activeStateChanged(self):
        if self.automataScene.activeState != self.activeState:
            print('visual states active state changed:' + self.automataScene.activeState.name)
            self.activeState = self.automataScene.activeState
            if self.activeState.id != 0:
                self.treeView.setCurrentIndex(self.treeModel.indexOf(self.treeModel.getByDataId(self.activeState.id)))

    def upButtonClicked(self):
        if self.activeState != None:
            if self.activeState.parent != None:
                print('parent name:' + self.activeState.parent.name)
                self.automataScene.setActiveState(self.activeState.parent)
