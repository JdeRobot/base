import rospy
from geometry_msgs.msg import Twist
import threading
from math import pi as PI
from jderobotTypes import CMDVel



def cmdvel2Twist(vel):
    '''
    Translates from JderobotTypes CMDVel to ROS Twist. 

    @param vel: JderobotTypes CMDVel to translate

    @type img: JdeRobotTypes.CMDVel

    @return a Twist translated from vel

    '''
    tw = Twist()
    tw.linear.x = vel.vx
    tw.linear.y = vel.vy
    tw.linear.z = vel.vz
    tw.angular.x = vel.ax
    tw.angular.y = vel.ay
    tw.angular.z = vel.az

    secs = int(vel.timeStamp)
    nsecs = int((vel.timeStamp-secs)* 1e9) # nanosecs
    tw.header.stamp.secs = secs
    tw.header.stamp.nsecs = nsecs
    return tw

class PublisherMotors:
    '''
        ROS Motors Publisher. Motors Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerMotors Constructor.

        @param topic: ROS topic to publish
        
        @type topic: String

        '''
        self.topic = topic
        self.data = CMDVel()
        self.pub = None
        self.lock = threading.Lock()
        self.start()
 
    def __publish (self):
        '''
        Function to publish cmdvel. 
        '''
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)
        
    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.pub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        

    def sendVelocities(self, vel):
        '''
        Sends CMDVel.

        @param vel: CMDVel to publish
        
        @type vel: CMDVel

        '''
        self.lock.acquire()
        self.data = vel
        self.__publish()
        self.lock.release()

    def sendV(self, v):
        '''
        Sends V velocity. uses self.sendVX

        @param v: V velocity
        
        @type v: float

        '''
        self.sendVX(v)

    def sendL(self, l):
        '''
        Sends L velocity. uses self.sendVY

        @param l: L velocity
        
        @type l: float

        '''
        self.sendVY(l)

    def sendW(self, w):
        '''
        Sends W velocity. uses self.sendAZ

        @param w: W velocity
        
        @type w: float

        '''
        self.sendAZ(w)

    def sendVX(self, vx):
        '''
        Sends VX velocity.

        @param vx: VX velocity
        
        @type vx: float

        '''
        self.lock.acquire()
        self.data.vx = vx
        self.__publish()
        self.lock.release()

    def sendVY(self, vy):
        '''
        Sends VY velocity.

        @param vy: VY velocity
        
        @type vy: float

        '''
        self.lock.acquire()
        self.data.vy = vy
        self.__publish()
        self.lock.release()

    def sendAZ(self, az):
        '''
        Sends AZ velocity.

        @param az: AZ velocity
        
        @type az: float

        '''
        self.lock.acquire()
        self.data.vz = vz
        self.__publish()
        self.lock.release()


