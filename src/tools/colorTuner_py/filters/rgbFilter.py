
from threading import Lock
import cv2
import numpy as np

RGBMAX = [255,255,255]
RGBMIN = [0,0,0]


class RgbFilter:

    def __init__(self):

        self.lock = Lock()

        self.MAX = RGBMAX
        self.MIN = RGBMIN

        self.uLimit = self.MAX
        self.dLimit = self.MIN

    def getName(self):
        return 'RGB'

    def setUpLimit (self, r, g, b):
        self.lock.acquire()
        self.uLimit = [r,g,b]
        self.lock.release()


    def getUpLimit (self):
        self.lock.acquire()
        lim = self.uLimit
        self.lock.release()
        return lim

    def setDownLimit(self, r, g, b):
        self.lock.acquire()
        self.dLimit = [r,g,b]
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


        '''rup,gup,bup = self.getUpLimit()
        rdwn,gdwn,bdwn = self.getDownLimit()

        rch, gch, bch = cv2.split(img)

        height = len(rch)
        width = len(rch[0])

        for i in range(height):
            for j in range (width):
                r = rch[i][j]
                g = gch[i][j]
                b = bch[i][j]

                condr = (r<=rup) and (r>=rdwn)
                condg = (g<=gup) and (g>=gdwn)
                condb = (b<=bup) and (b>=bdwn)

                if (not (condr and condb and condg)):
                    rch[i][j] = 0
                    gch[i][j] = 0
                    bch[i][j] = 0

        return cv2.merge([rch,gch,bch])'''
        return img