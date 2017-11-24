import rospy
from kobuki_msgs.msg import BumperEvent
import threading
from jderobotTypes import BumperData



def bumperEvent2BumperData(event):
    '''
    Translates from ROS BumperScan to JderobotTypes BumperData. 

    @param event: ROS BumperScan to translate

    @type event: BumperScan

    @return a BumperData translated from event

    # bumper
    LEFT   = 0
    CENTER = 1
    RIGHT  = 2

    #  state
    RELEASED = 0
    PRESSED  = 1

    '''
    bump = BumperData()
    bump.state = event.state
    bump.bumper = event.bumper
    
    #bump.timeStamp = event.header.stamp.secs + (event.header.stamp.nsecs *1e-9)
    return bump

class ListenerBumper:
    '''
        ROS Bumper Subscriber. Bumper Client to Receive Bumper Scans from ROS nodes.
    '''
    def __init__(self, topic):
        '''
        ListenerBumper Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        self.topic = topic
        self.data = BumperData()
        self.sub = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, event):
        '''
        Callback function to receive and save Bumper Scans. 

        @param event: ROS BumperScan received
        
        @type event: BumperScan

        '''
        bump = bumperEvent2BumperData(event)

        self.lock.acquire()
        self.data = bump
        self.lock.release()
        
    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.sub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.sub = rospy.Subscriber(self.topic, BumperEvent, self.__callback)
        
    def getBumperData(self):
        '''
        Returns last BumperData. 

        @return last JdeRobotTypes BumperData saved

        '''
        self.lock.acquire()
        bump = self.data
        self.lock.release()
        
        return bump


