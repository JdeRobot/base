from threading import Thread
from gui.runtimegui.python.runtimegui import RunTimeGui
from PyQt5.QtWidgets import QApplication
import sys

def runGui():
    app = QApplication(sys.argv)
    gui = RunTimeGui()
    gui.activateIPC()
    gui.show()
    app.exec_()

if __name__ == '__main__':
    runGui()