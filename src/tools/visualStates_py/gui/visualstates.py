from PyQt5.QtWidgets import QMainWindow, QAction, QDockWidget, QTreeView, QGraphicsView, QWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor
from gui.automatascene import AutomataScene, OpType
from gui.treemodel import TreeModel


class VisualStates(QMainWindow):
    def __init__(self, parent=None):
        super().__init__()

        self.setWindowTitle("VisualStates")

        # create status bar
        self.statusBar()

        self.createMenu()
        self.createTreeView()
        self.createStateCanvas()

        self.setGeometry(0, 0, 800, 600)
        self.show()

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
        archieveMenu = menubar.addMenu('&Archieve')
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
        print('New Action')
        pass

    def openAction(self):
        print('Open Action')
        pass

    def saveAction(self):
        print('Save Action')
        pass

    def saveAsAction(self):
        print('Save As Action')

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
        self.treeView.setColumnWidth(0, 50)
        dockWidget.setWidget(self.treeView)
        self.addDockWidget(Qt.LeftDockWidgetArea, dockWidget)

    def createStateCanvas(self):
        self.stateCanvas = QGraphicsView()
        self.automataScene = AutomataScene()
        self.automataScene.setSceneRect(0, 0, 2000, 2000)
        self.automataScene.stateInserted.connect(self.stateInserted)
        self.automataScene.transitionInserted.connect(self.transitionInserted)
        self.automataScene.stateNameChangedSignal.connect(self.stateNameChanged)
        self.stateCanvas.setScene(self.automataScene)
        self.setCentralWidget(self.stateCanvas)
        self.stateCanvas.setRenderHint(QPainter.Antialiasing)
        self.stateCanvas.setAcceptDrops(True)

    def stateInserted(self, state):
        print('state inserted:' + state.name)
        self.treeModel.insertState(state, QColor(Qt.white))

    def transitionInserted(self, tran):
        print('transition inserted:' + tran.name)

    def stateNameChanged(self, state):
        dataItem = self.treeModel.getByDataId(state.id)
        if dataItem != None:
            dataItem.name = state.name
            self.treeModel.layoutChanged.emit()
