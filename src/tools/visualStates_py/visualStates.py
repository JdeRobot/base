
import sys
from PyQt5.QtWidgets import QApplication
from gui.visualstates import VisualStates

if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainWindow = VisualStates()
    sys.exit(app.exec_())