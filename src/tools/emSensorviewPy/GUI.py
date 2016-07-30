from PyQt4 import QtCore
from PyQt4 import QtGui


class GUI(QtGui.QWidget):
    def __init__(self, sensor, parent=None):

        self.sensor = sensor
        self.running = True

        QtGui.QWidget.__init__(self, parent)

        self.InitUI();
        
    def InitUI(self):

        self.lblStatus = QtGui.QLabel();
        self.lblDistance = QtGui.QLabel();
        self.lblTimeStamp = QtGui.QLabel();
                            
        mainLayout = QtGui.QGridLayout()

        btnQuit = QtGui.QPushButton("Quit")
        btnQuit.clicked.connect(self.quit)
        
        wList =[
            [QtGui.QLabel("Status: "), self.lblStatus],
            [QtGui.QLabel("Distance: "), self.lblDistance],
            [QtGui.QLabel("TimeStamp: "), self.lblTimeStamp],
            [btnQuit]
            ]
        
        for i in range(len(wList)):
            for j in range(len(wList[i])):
                if wList[i][j] <> None:
                    mainLayout.addWidget(wList[i][j], i, j);

        self.setLayout(mainLayout)
        
        QtCore.QObject.connect(self, QtCore.SIGNAL("aa"), self.update)

        
    def quit(self):
        self.sensor.running = False
        self.running = False
        QtCore.QCoreApplication.instance().quit()

    def updateData(self):
        data = self.sensor.getData()
        self.lblStatus.setText(str(data.status))
        self.lblDistance.setText('{:.2f}'.format(data.d))
        self.lblTimeStamp.setText(str(data.tm))
        print data.__dict__
    
    def update(self):
        self.updateData()
        self.running = self.sensor.running
        
    def updateGUI_recieved_signal(self):
        self.emit(QtCore.SIGNAL("aa"), "lol")
