from PyQt5.QtWidgets import QGraphicsRectItem
from PyQt5.QtGui import QPen, QBrush, QPolygonF
from PyQt5.QtCore import Qt

SQUARE_SIDE = 10
PEN_FOCUS_WIDTH = 3
PEN_NORMAL_WIDTH = 1

class RectHandleGraphicsItem(QGraphicsRectItem):
    def __init__(self, width, parent=None):
        super().__init__(-SQUARE_SIDE / 2, -SQUARE_SIDE / 2, SQUARE_SIDE, SQUARE_SIDE, parent)
        self.setAcceptHoverEvents(True)

        # set the color of the rectangle
        brush = QBrush(Qt.SolidPattern)
        brush.setColor(Qt.red)
        self.setBrush(brush)

        self.dragging = False
        self.interaction = True

    def hoverEnterEvent(self, event):
        if self.interaction:
            myPen = QPen(Qt.SolidLine)
            myPen.setWidth(PEN_FOCUS_WIDTH)
            self.setPen(myPen)

    def hoverLeaveEvent(self, event):
        if self.interaction:
            myPen = QPen(Qt.SolidLine)
            myPen.setWidth(PEN_NORMAL_WIDTH)
            self.setPen(myPen)

    def mousePressEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = True
        super().mousePressEvent(qGraphicsSceneMouseEvent)

    def mouseReleaseEvent(self, qGraphicsSceneMouseEvent):
        if qGraphicsSceneMouseEvent.button() == Qt.LeftButton:
            self.dragging = False
        super().mouseReleaseEvent(qGraphicsSceneMouseEvent)

    def mouseMoveEvent(self, qGraphicsSceneMouseEvent):
        if self.dragging:
            self.parentItem().updateMiddlePoints(self.scenePos())
        super().mouseMoveEvent(qGraphicsSceneMouseEvent)

    def disableInteraction(self):
        self.interaction = False