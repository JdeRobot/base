import rospy
import message_filters
from sensor_msgs.msg import Image as ImageROS
import threading
from jderobotTypes import Rgbd, Image



def imageMsg2Image(img, bridge):
    '''
    Translates from ROS Image to JderobotTypes Image. 

    @param img: ROS Image to translate
    @param bridge: bridge to do translation

    @type img: sensor_msgs.msg.Image
    @type brige: CvBridge

    @return a JderobotTypes.Image translated from img

    '''
    image = Image()

    image.width = img.width
    image.height = img.height
    image.format = "RGB8"
    image.timeStamp = img.header.stamp.secs + (img.header.stamp.nsecs *1e-9)
    cv_image=0
    if (img.encoding == "32FC1"):
        gray_img_buff = bridge.imgmsg_to_cv2(img, "32FC1")
        cv_image  = depthToRGB8(gray_img_buff)
    else:
        cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
    image.data = cv_image
    return image



def Images2Rgbd(rgb, d):
    '''
    Translates from ROS Images to JderobotTypes Rgbd. 

    @param rgb: ROS color Image to translate

    @param d: ROS depth image to translate

    @type rgb: ImageROS

    @type d: ImageROS

    @return a Rgbd translated from Images

    '''
    data = Rgbd()
    data.color=imageMsg2Image(rgb)
    data.depth=imageMsg2Image(d)
    data.timeStamp = rgb.header.stamp.secs + (rgb.header.stamp.nsecs *1e-9)
    return data

class ListenerRgbd:
    '''
        ROS Rgbd Subscriber. Rgbd Client to Receive Rgbd Synchronized from ROS nodes.
    '''
    def __init__(self, topicrgb, topicd):
        '''
        ListenerRgbd Constructor.

        @param topicrgb: ROS topic for color image to subscribe

         @param topicd: ROS topic for depth image to subscribe
        
        @type topic: String

        '''
        self.topicrgb = topicrgb
        self.topicd = topicd
        self.data = Rbgd()
        self.subrgb = None
        self.subd = None
        self.ts = None
        self.lock = threading.Lock()
        self.start()
 
    def __callback (self, rgb, d):
        '''
        Callback function to receive and save Rgbd Scans. 

        @param rgb: ROS color Image to translate

        @param d: ROS depth image to translate

        @type rgb: ImageROS

        @type d: ImageROS

        '''
        data = Images2Rgbd(rgb, d)

        self.lock.acquire()
        self.data = data
        self.lock.release()
        
    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.subrgb.unregister()
        self.subd.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.subrgb = rospy.Subscriber(self.topicrgb, ImageROS)
        self.subd = rospy.Subscriber(self.topicd, ImageROS)
        self.ts = message_filters.ApproximateTimeSynchronizer([subrgb, subd], 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.__callback)
        
    def getRgbdData(self):
        '''
        Returns last RgbdData. 

        @return last JdeRobotTypes Rgbd saved

        '''
        self.lock.acquire()
        data = self.data
        self.lock.release()
        
        return data


