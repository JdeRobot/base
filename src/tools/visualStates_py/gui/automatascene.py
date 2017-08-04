from PyQt5.QtWidgets import QGraphicsScene, QGraphicsItem, QAction, QMenu
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QPoint
from enum import Enum
from . import guistate, guitransition
from .renamediaolog import RenameDialog
from .codedialog import CodeDialog


class AutomataScene(QGraphicsScene):
    # slots
    stateInserted = pyqtSignal('QGraphicsItem')
    stateRemoved = pyqtSignal('QGraphicsItem')
    transitionInserted = pyqtSignal('QGraphicsItem')
    transitionRemoved = pyqtSignal('QGraphicsItem')
    stateNameChangedSignal = pyqtSignal('QGraphicsItem')

    def __init__(self, parent=None):
        super().__init__(parent)

        self.operationType = None

        # transition origin and destination
        self.origin = None
        self.destination = None

        self.stateIndex = -1
        self.transitionIndex = -1

        self.prevOperationType = None
        self.stateTextEditingStarted = False

        # the active state whose children will be drawn on the graphicsview
        self.activeState = None

        self.createActions()

        self.selectedState = None
        self.contextPosition = None

        self.copiedState = None

    def createActions(self):

        self.renameStateAction = QAction('Rename', self)
        self.renameStateAction.triggered.connect(self.renameState)

        self.stateCodeAction = QAction('Code', self)
        self.stateCodeAction.triggered.connect(self.editStateCode)

        self.makeInitialAction = QAction('Make Initial', self)
        self.makeInitialAction.triggered.connect(self.makeInitial)

        self.copyStateAction = QAction('Copy', self)
        self.copyStateAction.triggered.connect(self.copyState)

        self.removeStateAction = QAction('Remove', self)
        self.removeStateAction.triggered.connect(self.removeState)

        self.pasteStateAction = QAction('Paste', self)
        self.pasteStateAction.triggered.connect(self.pasteState)


    def renameState(self):
        dialog = RenameDialog('Rename', self.selectedState.name)
        dialog.move(self.contextPosition)
        dialog.nameChanged.connect(self.externalStateNameChanged)
        dialog.exec_()

    def editStateCode(self, state):
        dialog = CodeDialog('State Code', self.selectedState.code)
        dialog.move(self.contextPosition)
        dialog.codeChanged.connect(self.codeChanged)
        dialog.exec_()

    def makeInitial(self):
        # there could be only one initial state
        for child in self.activeState.getChildren():
            child.setInitial(False)

        self.selectedState.setInitial(True)


    #TODO: do i need to copy also transitions?
    def copyState(self):
        self.copiedState = self.selectedState.getNewCopy()

    def pasteState(self):
        self.copiedState.id = self.getStateIndex()
        self.copiedState.setPos(self.currentScenePos)
        self.addStateItem(self.copiedState)
        self.copyState()

    def removeState(self):
        self.removeStateItem(self.selectedState)


    def mousePressEvent(self, qGraphicsSceneMouseEvent):
        super().mousePressEvent(qGraphicsSceneMouseEvent)


    # def mouseMoveEvent(self, qGraphicsSceneMouseEvent):
    #     super().mouseMoveEvent(qGraphicsSceneMouseEvent)
    #     selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
    #     if len(selectedItems) > 0:
    #         if isinstance(selectedItems[0], guistate.StateGraphicsItem):
    #             if selectedItems[0].dragging:
    #                 self.origin = None

    def addTransitionItem(self, tranItem):
        self.addItem(tranItem)
        self.transitionInserted.emit(tranItem)


    def addStateItem(self, stateItem):
        stateItem.stateNameChanged.connect(self.stateNameChanged)
        stateItem.stateTextEditStarted.connect(self.stateTextEditStarted)
        stateItem.stateTextEditFinished.connect(self.stateTextEditFinished)

        self.addItem(stateItem)
        self.activeState.addChild(stateItem)
        self.stateInserted.emit(stateItem)

    def removeStateItem(self, stateItem):
        stateItem.stateNameChanged.disconnect(self.stateNameChanged)
        stateItem.stateTextEditStarted.disconnect(self.stateTextEditStarted)
        stateItem.stateTextEditFinished.disconnect(self.stateTextEditFinished)

        for tran in stateItem.getOriginTransitions():
            print('removing origin:' + tran.name)
            tran.destination.removeTargetTransition(tran)
            self.removeItem(tran)
            self.transitionRemoved.emit(tran)

        for tran in stateItem.getTargetTransitions():
            print('removing destination:' + tran.name)
            tran.origin.removeOriginTransition(tran)
            self.removeItem(tran)
            self.transitionRemoved.emit(tran)

        stateItem.removeTransitions()

        if self.origin == stateItem:
            self.origin = None

        self.removeItem(stateItem)
        self.activeState.removeChild(stateItem)

        self.stateRemoved.emit(stateItem)

    def removeTransitionItem(self, tranItem):
        tranItem.origin.removeOriginTransition(tranItem)
        tranItem.destination.removeTargetTransition(tranItem)
        print('removing tran:' + tranItem.name)



    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):

        # if we were editing the state text next mouse release should disable text editing and should not add a new state or transition
        if self.stateTextEditingStarted:
            self.stateTextEditingStarted = False
            super().mouseReleaseEvent(qGraphicsSceneMouseEvent)
            return

        if self.operationType == OpType.ADDSTATE and qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            if len(selectedItems) == 0:
                sIndex = self.getStateIndex()
                stateItem = guistate.StateGraphicsItem(sIndex, qGraphicsSceneMouseEvent.scenePos().x(),
                                                       qGraphicsSceneMouseEvent.scenePos().y(), False,
                                                       'state ' + str(sIndex))
                self.addStateItem(stateItem)

            self.origin = None
        elif self.operationType == OpType.ADDTRANSITION and qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            if len(selectedItems) > 0:
                # get the parent
                item = self.getParentItem(selectedItems[0])
                if isinstance(item, guistate.StateGraphicsItem):
                    if self.origin != None:
                        self.destination = item
                        tIndex = self.getTransitionIndex()
                        tranItem = guitransition.TransitionGraphicsItem(self.origin, self.destination, tIndex,
                                                                        'transition ' + str(tIndex))
                        self.addTransitionItem(tranItem)
                    else:
                        self.origin = item
                else:
                    self.origin = None
            else:
                self.origin = None

        super().mouseReleaseEvent(qGraphicsSceneMouseEvent)

    def contextMenuEvent(self, qGraphicsSceneContextMenuEvent):
        super().contextMenuEvent(qGraphicsSceneContextMenuEvent)
        selectedItems = self.items(qGraphicsSceneContextMenuEvent.scenePos())
        if len(selectedItems) > 0:
            item = self.getParentItem(selectedItems[0])
            if isinstance(item, guistate.StateGraphicsItem):
                self.showStateContextMenu(item, qGraphicsSceneContextMenuEvent)
            elif isinstance(item, guitransition.TransitionGraphicsItem):
                self.showTransitionContextMenu(item, qGraphicsSceneContextMenuEvent)
        elif len(selectedItems) == 0:
            self.showSceneContextMenu(qGraphicsSceneContextMenuEvent)

    def mouseDoubleClickEvent(self, qGraphicsSceneMouseEvent):
        super().mouseDoubleClickEvent(qGraphicsSceneMouseEvent)
        selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
        if len(selectedItems) == 1:
            item = self.getParentItem(selectedItems[0])
            if isinstance(item, guistate.StateGraphicsItem):
                #TODO: now change the active state
                pass



    def showStateContextMenu(self, stateItem, qEvent):
        cMenu = QMenu()
        cMenu.addAction(self.renameStateAction)
        cMenu.addAction(self.stateCodeAction)
        cMenu.addAction(self.makeInitialAction)
        cMenu.addAction(self.copyStateAction)
        cMenu.addAction(self.removeStateAction)
        self.selectedState = stateItem
        self.contextPosition = qEvent.screenPos()
        action = cMenu.exec_(qEvent.screenPos())

    def showTransitionContextMenu(self, tranItem, qEvent):
        pass

    def showSceneContextMenu(self, qEvent):
        cMenu = QMenu()
        cMenu.addAction(self.pasteStateAction)
        self.currentScenePos = qEvent.scenePos()
        action = cMenu.exec_(qEvent.screenPos())

    def setOperationType(self, type):
        self.operationType = type

    def getStateIndex(self):
        self.stateIndex += 1
        return self.stateIndex

    def getTransitionIndex(self):
        self.transitionIndex += 1
        return self.transitionIndex

    def getParentItem(self, item):
        while item.parentItem() is not None:
            item = item.parentItem()
        return item


    def externalStateNameChanged(self, newName):
        self.selectedState.textGraphics.setPlainText(newName)
        self.selectedState.textGraphics.textChanged.emit(newName)

    def codeChanged(self, newCode):
        self.selectedState.code = newCode

    def stateNameChanged(self, state):
        self.stateNameChangedSignal.emit(state)

    def stateTextEditStarted(self):
        # temporarily disable operation type while editing the text
        self.prevOperationType = self.operationType
        self.operationType = None
        self.stateTextEditingStarted = True
        print('text editing started')

    def stateTextEditFinished(self):
        # after text edit finishes restore operation type
        self.operationType = self.prevOperationType
        self.prevOperationType = None
        self.stateTextEditingStarted = True

    def removeAllItems(self):
        if self.activeState is not None:
            for s in self.activeState.getChildren():
                for t in s.getTransitions():
                    if t in self.items():
                        self.removeItem(t)
                if s in self.items():
                    self.removeItem(s)

    def setActiveState(self, state):
        if state != self.activeState:
            self.removeAllItems()
            self.activeState = state

    def resetIndexes(self):
        self.stateIndex = -1
        self.transitionIndex = -1


class OpType(Enum):
    ADDSTATE = 0
    ADDTRANSITION = 1
