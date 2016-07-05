__author__ = 'AeroCano'

import jderobot, time, threading

lock = threading.Lock()

class Pose3DI(jderobot.Pose3D):

    def __init__(self,_x,_y,_z,_h,_q0,_q1,_q2,_q3):

        self.x = _x
        self.y = _y
        self.z = _z
        self.h = _h
        self.q0 = _q0
        self.q1 = _q1
        self.q2 = _q2
        self.q3 = _q3

    def setPose3DData(self,_x,_y,_z,_h,_q0,_q1,_q2,_q3):

        lock.acquire()
        #print 'adquired S'

        self.x = _x
        self.y = _y
        self.z = _z
        self.h = _h
        self.q0 = _q0
        self.q1 = _q1
        self.q2 = _q2
        self.q3 = _q3

        #print 'released S'
        lock.release()

        return 0



    def getPose3DData(self, current=None):

        time.sleep(0.05) #50 ms rate to tx Pose3D

        lock.acquire()
        #print 'adquired G'

        data = jderobot.Pose3DData()
        data.x = self.x
        data.y = self.y
        data.z = self.z
        data.h = self.h
        data.q0 = self.q0
        data.q1 = self.q1
        data.q2 = self.q2
        data.q3 = self.q3

        #print 'released G'
        lock.release()

        return data