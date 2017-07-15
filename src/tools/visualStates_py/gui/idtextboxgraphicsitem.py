
from PyQt5.QtWidgets import QGraphicsTextItem
from PyQt5.QtCore import Qt

class IdTextBoxGraphicsItem(QGraphicsTextItem):
    def __init__(self, name, parent=None):
        super().__init__(name, parent)
        self.name = name


    def mouseDoubleClickEvent(self, event):
        if self.textInteractionFlags() == Qt.NoTextInteraction:
            self.setTextInteractionFlags(Qt.TextEditorInteraction)

        super().mouseDoubleClickEvent(event)


    def focusOutEvent(self, event):
        self.setTextInteractionFlags(Qt.NoTextInteraction)
        super().focusOutEvent(event)
