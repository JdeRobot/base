from PyQt5.QtWidgets import QTreeWidgetItem
from PyQt5.QtGui import QColor

class TreeNode(QTreeWidgetItem):
    def __init__(self, id, name, color, parent=None):
        super().__init__(parent)

        self.parentItem = parent
        self.id = id
        self.name = name
        self.color = color
        self.childItems = []

    def background(self):
        background = QColor(self.color)
        return background

    def appendChild(self, item):
        self.childItems.append(item)

    def removeChild(self, item):
        self.childItems.remove(item)

    def child(self, row):
        return self.childItems[row]

    def childCount(self):
        return len(self.childItems)

    def columnCount(self):
        return 2

    def data(self, column):
        if column == 0:
            return self.id
        if column == 1:
            return self.name

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem:
            return self.parentItem.childItems.index(self)
        return 0

    def getChildren(self):
        return self.childItems

    def setColor(self, color):
        self.color = color

    def removeChildren(self):
        self.childItems.clear()