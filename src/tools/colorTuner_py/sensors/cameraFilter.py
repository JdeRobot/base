#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Alberto Martin Florido <almartinflorido@gmail.com>
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#
import numpy as np
import threading
from parallelIce.cameraClient import CameraClient
from parallelIce.threadSensor import ThreadSensor
import cv2

DEGTORAD = (3.14159264 / 180.0)

RADTODEG = (180.0 /3.14159264)


class CameraFilter:

    def __init__(self, camera):
        self.lock = threading.Lock()
        self.client = camera

        self.height= self.client.getHeight()
        self.width = self.client.getWidth()


        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self, self.kill_event)
        self.thread.daemon = True

        self.rgbLimits = RGBLimits()
        self.yuvLimits = YUVLimits()
        self.hsvLimits = HSVLimits()

        if self.client.hasproxy():
             
            trackImage = np.zeros((self.height, self.width,3), np.uint8)
            trackImage.shape = self.height, self.width, 3

            self.images = {'Orig': trackImage, 'RGB': trackImage, 'HSV': trackImage, 'YUV': trackImage} 

            self.kill_event.clear()
            self.thread.start()

            #self.images['Orig'].shape =  

    # if client is stopped you can not start again, Threading.Thread raised error
    def stop(self):
        self.kill_event.set()
        self.client.stop()
    
    
    def getImage(self):
        return getOrigImage()

    
    def getFilteredImage(self, filt):
        if self.client.hasproxy():
            img = np.zeros((self.height, self.width,3), np.uint8)
            self.lock.acquire()
            img = self.images[filt]
            img.shape = self.images[filt].shape
            self.lock.release()
            return img
        return None

    def getOrigImage(self):
        return self.getFilteredImage("Orig")

    def getHSVImage (self):
        return self.getFilteredImage("HSV")

    def getRGBImage (self):
        return self.getFilteredImage("RGB")

    def getYUVImage (self):
        return self.getFilteredImage("YUV")


    def update(self):
        img = self.client.getImage()
        rgb = self.filterRGB(img)
        hsv = self.filterHSV(img)
        yuv = self.filterYUV(img)

        self.lock.acquire()
        print("hmax: " + str(self.hsvLimits.hmax) + " hmin: " + str(self.hsvLimits.hmin) + " smax: " + str(self.hsvLimits.smax) + " smin: " + str(self.hsvLimits.smin) + " vmax: " + str(self.hsvLimits.vmax) + " vmin: " + str(self.hsvLimits.vmin))

        self.images['Orig'] = img
        self.images['RGB'] = rgb
        self.images['HSV'] = hsv
        self.images['YUV'] = yuv
        self.lock.release()


    def setRGBLimits (self, rgbLimits):
        self.lock.acquire()
        self.rgbLimits = rgbLimits
        self.lock.release()


    def setHSVLimits (self, hsvLimits):
        self.lock.acquire()
        self.hsvLimits = hsvLimits
        self.lock.release()

    def setYUVLimits (self, yuvLimits):
        self.lock.acquire()
        self.yuvLimits = yuvLimits
        self.lock.release()


    def filterRGB(self, img):
        return img

    def filterHSV(self, img):
        self.lock.acquire()
        hsvLimits = self.hsvLimits
        self.lock.release()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        print("Dtype img: " + str(img.shape))
        
        minValues = np.array([round(hsvLimits.hmin*255/6),round(hsvLimits.smin*255/1),hsvLimits.vmin])
        maxValues = np.array([round(hsvLimits.hmax*255/6),round(hsvLimits.smax*255/1),hsvLimits.vmax])

        mask = cv2.inRange(hsv, minValues, maxValues)

        res = cv2.bitwise_and(img,img, mask= mask)

        print("Dtype res: " + str(res.shape))

        return res


        #hsvLimits = self.hsvLimits

        #for i in img.data:
        #    r ,g, b =  i
        #    h = self.getH(r,g,b)
        #    s = self.getS(r,g,b)
        #    v = self.getV(r,g,b)
        #    if (not(hsvLimits.hmax >= h* DEGTORAD and hsvLimits.hmin <= h*DEGTORAD and hsvLimits.smax >= s and hsvLimits.smin <= s and  hsvLimits.vmax >= v and hsvLimits.vmin <= v)):

        #"""





        

    def filterYUV(self, img):
        return img

    """  def getH (r,g,b):
        max = 0.0
        min = 255.0

        if(r >= g and r >= b):
            max =  r
        if( g >= r and g >= b ):    
            max =  g
        if(b >= r and b >= g):
            max = b
            
        if(r <= g and r <= b):
            min =  r
        if( g <= r and g <= b ):    
            min =  g
        if(b <= r and b <= g):
            min = b
            
        if(max == min)
            return 0
            
        if(max == r):
            if(g>=b):
                return (60.0*(g-b)/(max-min))
            else:
                return ((60.0*(g-b)/(max-min))+360.0)
        }
        if(max == g):
            return ((60.0*(b-r)/(max-min))+120.0)

        if(max == b )
            return ((60.0*(r-g)/(max-min))+240.0) 
        
        return 0;

    def getS(r, g, b):
        double max = 0.0
        double min = 255.0

        if(r >= g and r >= b):
            max =  r
        if( g >= r and g >= b ):    
            max =  g
        if(b >= r and b >= g):
            max = b
        if(max == 0.0):
            return 0.0    
        if(r <= g and r <= b):
            min =  r
        if( g <= r and g <= b ):    
            min =  g
        if(b <= r and b <= g):
            min = b

        return (1.0 - (min/max))

    def getV(r, g, b):
        if(r >= g and r >= b):
            return  r
        if( g >= r and g >= b ):    
            return  g
        if(b >= r and b >= g):
            return b
            
        return 0
    """


class RGBLimits:
    def __init__(self):
        self.rmax = 255
        self.rmin = 0
        self.gmax = 255
        self.gmin = 0
        self.bmax = 255
        self.bmin = 0

class HSVLimits:
    def __init__(self):
        self.hmax = 6.0
        self.hmin = 0
        self.smax = 1.0
        self.smin = 0
        self.vmax = 255
        self.vmin = 0


class YUVLimits:
    def __init__(self):
        self.ymax = 0
        self.ymin = 0
        self.umax = 0
        self.umin = 0
        self.vmax = 0
        self.vmin = 0