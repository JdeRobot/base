#
#  Copyright (C) 1997-2017 JDE Developers Team
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
from jderobotTypes import Rgbd


class RgbdCamera:
    '''
        Rgbd Connector. Recives image from Ice interface when you run update method.
    '''

    def __init__(self, jdrc, prefix):
        '''
        Rgbd Contructor.
        Exits When it receives a Exception diferent to Ice.ConnectionRefusedException

        @param jdrc: Comm Communicator
        @param prefix: Name of client in config file

        @type jdrc: Comm Communicator
        @type prefix: String
        '''

        self.lock = threading.Lock()
        self.image = Image()

        try:

            ic = jdrc.getIc()
            proxyStr = jdrc.getConfig().getProperty(prefix+".Proxy")
            basecamera = ic.stringToProxy(proxyStr)
            self.proxy = jderobot.RgbdPrx.checkedCast(basecamera)
            #self.imgFormat = jdrc.getConfig().getProperty(prefix+".Format")
            #if not self.imgFormat:
            #    self.imgFormat = "RGB8"

            self.update()

            if not self.proxy:
                print ('Interface ' + prefix + ' not configured')

        except Ice.ConnectionRefusedException:
            print(prefix + ': connection refused')

        except:
            traceback.print_exc()
            exit(-1)

    def update(self):
        '''
        Updates Image.
        '''
        if self.hasproxy():
            img = Rgbd()
            data = self.proxy.getData()
            img.color.data = np.frombuffer(data.color.pixelData, dtype=np.uint8)
            img.color.data.shape =  data.color.description.height, data.color.description.width, 3
            img.color.height = data.color.description.height
            img.color.width = data.color.description.width
            img.color.format = data.color.description.format

            img.depth.data = np.frombuffer(data.depth.pixelData, dtype=np.uint8)
            img.depth.data.shape =  data.depth.description.height, data.depth.description.width, 3
            img.depth.height = data.depth.description.height
            img.depth.width = data.depth.description.width
            img.depth.format = data.depth.description.format

            img.timeStamp = data.timeStamp.seconds + data.timeStamp.useconds * 1e-9


            self.lock.acquire()
            self.image = img
            self.lock.release()

    def getRgbd(self):
        '''
        Returns last Rgbd. 

        @return last JdeRobotTypes Rgbd saved

        '''
        img = Rgb()      
        if self.hasproxy():
            self.lock.acquire()
            img = self.image
            self.lock.release()
        return img

    def hasproxy (self):
        '''
        Returns if proxy has ben created or not. 

        @return if proxy has ben created or not (Boolean)

        '''
        return hasattr(self,"proxy") and self.proxy




class RgbdIceClient:
    '''
        Rgbd Ice Client. Recives image from Ice interface running camera update method in a thread.
    '''
    def __init__(self,jdrc,prefix, start = False):
        '''
        RgbdIceClient Contructor.

        @param jdrc: Comm Communicator
        @param prefix: Name of client in config file
        @param start: indicates if start automatically the client

        @type jdrc: Comm Communicator
        @type prefix: String
        @type start: Boolean
        '''
        self.camera = RgbdCamera(jdrc,prefix)

        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self.camera, self.kill_event)
        #self.thread = ThreadSensor(self.camera)
        self.thread.daemon = True

        if start:
            self.start()

    
    def start(self):
        '''
        Starts the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.clear()
        self.thread.start()

    
    def stop(self):
        '''
        Stops the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.set()

    def getRgbd(self):
        '''
        Returns last Image. 

        @return last JdeRobotTypes Rgbd saved

        '''
        return self.camera.getRgbd()


    def hasproxy (self):
        '''
        Returns if proxy has ben created or not. 

        @return if proxy has ben created or not (Boolean)

        '''
        return self.camera.hasproxy()
