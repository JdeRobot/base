from PyQt5.QtWidgets import QGraphicsScene, QGraphicsItem
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from enum import Enum
from . import guistate, guitransition


class AutomataScene(QGraphicsScene):
    # slots
    stateInserted = pyqtSignal('QGraphicsItem')
    transitionInserted = pyqtSignal('QGraphicsItem')
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

    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):

        # if we were editing the state text next mouse release should disable text editing and should not add a new state or transition
        if self.stateTextEditingStarted:
            self.stateTextEditingStarted = False
            super().mouseReleaseEvent(qGraphicsSceneMouseEvent)
            print('state text editing ended')
            return

        if self.operationType == OpType.ADDSTATE and qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            print('now add state')
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            if len(selectedItems) == 0:
                sIndex = self.getStateIndex()
                stateItem = guistate.StateGraphicsItem(sIndex, qGraphicsSceneMouseEvent.scenePos().x(),
                                                       qGraphicsSceneMouseEvent.scenePos().y(), False,
                                                       'state ' + str(sIndex))
                self.addStateItem(stateItem)

            self.origin = None
        elif self.operationType == OpType.ADDTRANSITION and qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            print('now add transition')
            selectedItems = self.items(qGraphicsSceneMouseEvent.scenePos())
            print('selectedItems.len=' + str(len(selectedItems)))
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
