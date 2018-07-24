import rospy
from mavros_msgs.srv import CommandBool,CommandTOL, SetMode 
from geometry_msgs.msg import TwistStamped
import threading
import time

class PublisherExtra:
    '''
        ROS CMDVel Publisher. CMDVel Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topicArming, topicLand, topicSetMode, topicVel, jdrc):
        '''
        PublisherCMDVel Constructor.

        @param topicArming: ROS topic of arming service
        @param topicLand: ROS topic of land service
        @param topicSetMode: ROS topic of setMode service
        @param jdrc: jderobot Communicator
        
        @type topicArming: String
        @type topicLand: String
        @type topicSetMode: String
        @type jdrc: jderobot Communicator

        '''
        self.topicArming = topicArming
        self.topicLand = topicLand
        self.topicSetMode = topicSetMode
        self.jdrc = jdrc

        self.lock = threading.Lock()


        self.arming_client = rospy.ServiceProxy(topicArming,CommandBool)
        self.land_client = rospy.ServiceProxy(topicLand,CommandTOL)        
        self.set_mode_client = rospy.ServiceProxy(topicSetMode,SetMode)
        self.vel = self.vel = rospy.Publisher(topicVel, TwistStamped, queue_size=1)
        

        

    def takeoff(self):
        if (self.jdrc.getState()!= "flying"):
            self.jdrc.setState("takingoff")
            self.lock.acquire()
            self.arming_client.call(value=True)
            time.sleep(0.5)
            self.set_mode_client.call(custom_mode="OFFBOARD")
            tw = TwistStamped()
            tw.twist.linear.z = 1
            for i in range(20):
            	self.vel.publish(tw)
            	time.sleep(0.1)
            tw = TwistStamped()
            self.vel.publish(tw)
            self.lock.release()
            self.jdrc.setState("flying")
        
    def land(self):
        self.lock.acquire()
        self.land_client.call(0,0,0,0,0)
        self.lock.release()
        self.jdrc.setState("landed")
        
    def toggleCam(self):
        pass
              
    def reset(self):
        pass
        
    def record(self,record):
        pass


