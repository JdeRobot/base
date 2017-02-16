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
#       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
#

import traceback
import jderobot
import numpy as np
import threading
import Ice
from .threadSensor import ThreadSensor
from jderobotTypes import Image


class Camera:

    def __init__(self, ic, prefix):
        self.lock = threading.Lock()
        self.image = Image()

        try:
            basecamera = ic.propertyToProxy(prefix+".Proxy")
            self.proxy = jderobot.CameraPrx.checkedCast(basecamera)
            prop = ic.getProperties()
            self.imgFormat = prop.getProperty(prefix+".Format")
            if not self.imgFormat:
                self.imgFormat = "RGB8"

            self.update()

            if not self.proxy:
                print ('Interface ' + prefix + ' not configured')

        except Ice.ConnectionRefusedException:
            print(prefix + ': connection refused')

        except:
            traceback.print_exc()
            exit(-1)

    def update(self):
        if self.hasproxy():
            img = Image()
            imageData = self.proxy.getImageData(self.imgFormat)
            img.height = imageData.description.height
            img.width = imageData.description.width
            img.format = imageData.description.format

            img.data = np.frombuffer(imageData.pixelData, dtype=np.uint8)
            img.data.shape = img.height, img.width, 3

            img.timeStamp = imageData.timeStamp.seconds + imageData.timeStamp.useconds * 1e-9


            self.lock.acquire()
            self.image = img
            self.lock.release()

    def getImage(self):
        img = Image()      
        if self.hasproxy():
            self.lock.acquire()
            img = self.image
            self.lock.release()
        return img

    def hasproxy (self):
        return hasattr(self,"proxy") and self.proxy




class CameraIceClient:
    def __init__(self,ic,prefix, start = False):
        self.camera = Camera(ic,prefix)

        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self.camera, self.kill_event)
        #self.thread = ThreadSensor(self.camera)
        self.thread.daemon = True

        if start:
            self.start()

    # if client is stopped you can not start again, Threading.Thread raised error
    def start(self):
        self.kill_event.clear()
        self.thread.start()

    # if client is stopped you can not start again
    def stop(self):
        self.kill_event.set()

    def getImage(self):
        return self.camera.getImage()


    def hasproxy (self):
        return self.camera.hasproxy()
