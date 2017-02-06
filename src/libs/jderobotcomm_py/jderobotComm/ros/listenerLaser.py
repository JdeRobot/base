import rospy
from sensor_msgs.msg import LaserScan
import threading
from math import pi as PI
from jderobotTypes import LaserData



def laserScan2LaserData(scan):
    laser = LaserData()
    laser.values = scan.ranges
    laser.minAngle = scan.angle_min  - PI/2
    laser.maxAngle = scan.angle_max  - PI/2
    laser.maxRange = scan.range_max
    laser.minRange = scan.range_min
    laser.timeStamp = scan.header.stamp.secs + (scan.header.stamp.nsecs *1e-9)
    return laser

class ListenerLaser:
    def __init__(self, topic):
        self.topic = topic
        self.data = LaserData()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, scan):
        laser = laserScan2LaserData(scan)

        self.lock.acquire()
        self.data = laser
        self.lock.release()
        
    def stop(self):
        self.sub.unregister()

    def start (self):
        self.sub = rospy.Subscriber(self.topic, LaserScan, self.__callback)
        
    def getLaserData(self):
        self.lock.acquire()
        laser = self.data
        self.lock.release()
        
        return laser


