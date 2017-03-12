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

        print ("Pose3D start")

    def setPose3DData(self, data, current=None):

        lock.acquire()

        self.x = data.x
        self.y = data.y
        self.z = data.z
        self.h = data.h
        self.q0 = data.q0
        self.q1 = data.q1
        self.q2 = data.q2
        self.q3 = data.q3

        lock.release()

        return 0

    def getPose3DData(self, current=None):

        time.sleep(0.05) # 20Hz (50ms) rate to tx Pose3D

        lock.acquire()

        data = jderobot.Pose3DData()
        data.x = self.x
        data.y = self.y
        data.z = self.z
        data.h = self.h
        data.q0 = self.q0
        data.q1 = self.q1
        data.q2 = self.q2
        data.q3 = self.q3

        lock.release()

        return data
