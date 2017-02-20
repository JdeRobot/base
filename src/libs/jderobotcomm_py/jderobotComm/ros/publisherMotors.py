import rospy
from geometry_msgs.msg import Twist
import threading
from math import pi as PI
from jderobotTypes import CMDVel



def cmdvel2Twist(vel):
    tw = Twist()
    tw.linear.x = vel.vx
    tw.linear.y = vel.vy
    tw.linear.z = vel.vz
    tw.angular.x = vel.ax
    tw.angular.y = vel.ay
    tw.angular.z = vel.az
    #tw.timeStamp = scan.header.stamp.secs + (scan.header.stamp.nsecs *1e-9)
    return tw

class PublisherMotors:
    def __init__(self, topic):
        self.topic = topic
        self.data = CMDVel()
        self.pub = None
        self.lock = threading.Lock()
        self.start()
 
    def __publish (self):
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)
        
    def stop(self):
        self.pub.unregister()

    def start (self):
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        

    def sendVelocities(self, vel):
        self.lock.acquire()
        self.data = vel
        self.__publish()
        self.lock.release()

    def sendV(self, v):
        self.sendVX(v)

    def sendL(self, l):
        self.sendVY(l)

    def sendW(self, w):
        self.sendAZ(w)

    def sendVX(self, vx):
        self.lock.acquire()
        self.data.vx = vx
        self.__publish()
        self.lock.release()

    def sendVY(self, vy):
        self.lock.acquire()
        self.data.vy = vy
        self.__publish()
        self.lock.release()

    def sendAZ(self, az):
        self.lock.acquire()
        self.data.vz = vz
        self.__publish()
        self.lock.release()


