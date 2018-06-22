import rospy
from mavros_msgs.srv import CommandBool,CommandTOL, SetMode 
import threading
import time



class PublisherExtra:
    '''
        ROS CMDVel Publisher. CMDVel Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topicTakeOff, topicLand, topicSetMode):
        '''
        PublisherCMDVel Constructor.

        @param topic: ROS topic to publish
        
        @type topic: String

        '''
        self.topicTakeOff = topicTakeOff
        self.topicLand = topicLand
        self.topicSetMode = topicSetMode

        self.lock = threading.Lock()

        print("TAKEOFFF")


        #self.offb_set_mode = SetMode()
        #self.offb_set_mode.request.custom_mode = "OFFBOARD"
        
        #self.arm_cmd = CommandBool()
        #self. arm_cmd.request.value = True
        
        #self.land_cmd = CommandTOL()
        #self.land_cmd.request.yaw = 0
        #self.land_cmd.request.latitude = 0
        #self.land_cmd.request.longitude = 0
        #self.land_cmd.request.altitude = 0

        self.arming_client = rospy.ServiceProxy(topicTakeOff,CommandBool)
        self.land_client = rospy.ServiceProxy(topicLand,CommandTOL)        
        self.set_mode_client = rospy.ServiceProxy(topicSetMode,SetMode)
        self.takeoff1 = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        

    def takeoff(self):
        print("###########################################")
        self.lock.acquire()
        self.arming_client.call(value=True)
        time.sleep(5)
        
        print("OFFBOARD")
        self.set_mode_client.call(custom_mode="OFFBOARD")

        #self.takeoff1(altitude=100, latitude=0, longitude=0, min_pitch=0, yaw=0)
        #time.sleep(0.5)
        
        
        self.lock.release()
        
    def land(self):
        self.lock.acquire()
        #self.land_client.call(yaw=0,latitude=0, longitude=0, altitude=0)
        self.set_mode_client.call(custom_mode="AUTO.LAND")
        self.lock.release()
        
    def toggleCam(self):
        pass
              
    def reset(self):
        pass
        
    def record(self,record):
        pass


