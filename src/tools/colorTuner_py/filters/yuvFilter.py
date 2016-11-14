
from threading import Lock



class YuvFilter:

    def __init__(self):

        self.lock = Lock()

        self.MAX = [255,255,255]
        self.MIN = [0,0,0]

        self.uLimit = self.MAX
        self.dLimit = self.MIN

    def getName(self):
        return 'YUV'

    def setUpLimit (self, y, u, v):
        self.lock.acquire()
        self.uLimit = [y,u,v]
        self.lock.release()


    def getUpLimit (self):
        self.lock.acquire()
        lim = self.uLimit
        self.lock.release()
        return lim

    def setDownLimit(self, y, u, v):
        self.lock.acquire()
        self.dLimit = [y,u,v]
        self.lock.release()

    def getDownLimit(self):
        self.lock.acquire()
        lim = self.dLimit
        self.lock.release()
        return lim

    def getMAX(self):
        return self.MAX

    def getMIN(self):
        return self.MIN

    def apply (self, img):

        return img