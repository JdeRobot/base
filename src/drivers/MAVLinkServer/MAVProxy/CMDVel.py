__author__ = 'AeroCano'

import jderobot, time, threading

lock = threading.Lock()

class CMDVelI(jderobot.CMDVel):

    def __init__(self,lx,ly,lz,ax,ay,az):

        self.linearX = lx
        self.linearY = ly
        self.linearZ = lz
        self.angularX = ax
        self.angularY = ay
        self.angularZ = az

        #print ("cmdvel start")

    #def __del__(self):

        #print ("cmdvel end")


    def setCMDVelData(self, data, current=None):

        lock.acquire()

        self.linearX = data.linearX
        self.linearY = data.linearY
        self.linearZ = data.linearZ
        self.angularX = data.angularX
        self.angularY = data.angularY
        self.angularZ = data.angularZ

        lock.release()

        return 0

    def getCMDVelData(self, current=None):

        time.sleep(0.05)  # 20Hz (50ms) rate to rx CMDVel

        lock.acquire()

        data = jderobot.CMDVelData()
        data.linearX = self.linearX
        data.linearY = self.linearY
        data.linearZ = self.linearZ
        data.angularX = self.angularX
        data.angularY = self.angularY
        data.angularZ = self.angularZ
        lock.release()

        return data
