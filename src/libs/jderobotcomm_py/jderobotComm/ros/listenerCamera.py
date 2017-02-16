import rospy
from sensor_msgs.msg import Image as ImageROS
import threading
from math import pi as PI
from jderobotTypes import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError




def imageMsg2Image(img, bridge):
    image = Image()

    image.width = img.width
    image.height = img.height
    image.format = "RGB8"
    image.timeStamp = img.header.stamp.secs + (img.header.stamp.nsecs *1e-9)
    cv_image = bridge.imgmsg_to_cv2(image, "rgb8")
    image.data = cv_image
    return image

class ListenerCamera:
    def __init__(self, topic):
        self.topic = topic
        self.data = Image()
        self.sub = None
        self.lock = threading.Lock()

        self.bridge = CvBridge()
        self.start()
 
    def __callback (self, img):
        image = imageMsg2Image(img, self.bridge)

        self.lock.acquire()
        self.data = image
        self.lock.release()
        
    def stop(self):
        self.sub.unregister()

    def start (self):
        self.sub = rospy.Subscriber(self.topic, ImageROS, self.__callback)
        
    def getLaserData(self):
        self.lock.acquire()
        image = self.data
        self.lock.release()
        
        return image


