#
#  Copyright (C) 1997-2016 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY without even the implied warranty of
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
import threading
import Ice
from .threadSensor import ThreadSensor
from jderobotTypes import NavdataData


class NavData:

    def __init__(self, jdrc, prefix):
        self.lock = threading.Lock()

        try:
            ic = jdrc.getIc()
            proxyStr = jdrc.getConfig().getProperty(prefix+".Proxy")
            base = ic.stringToProxy(proxyStr)
            self.proxy = jderobot.NavdataPrx.checkedCast(base)

            self.navData = NavdataData()

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
            localNavdata = self.proxy.getNavdata()
            navdataData = NavdataData()

            navdataData.vehicle = localNavdata.vehicle
            navdataData.state = localNavdata.state
            
            navdataData.batteryPercent = localNavdata.batteryPercent
            navdataData.magX = localNavdata.magX
            navdataData.magY = localNavdata.magY
            navdataData.magZ = localNavdata.magZ
            navdataData.pressure = localNavdata.pressure
            navdataData.temp = localNavdata.temp
            
            navdataData.windSpeed = localNavdata.windSpeed
            navdataData.windAngle = localNavdata.windAngle
            navdataData.windCompAngle = localNavdata.windCompAngle
            
            navdataData.rotX = localNavdata.rotX
            navdataData.rotY = localNavdata.rotY
            navdataData.rotZ = localNavdata.rotZ
            navdataData.altd = localNavdata.altd

            navdataData.vx = localNavdata.vx
            navdataData.vy = localNavdata.vy
            navdataData.vz = localNavdata.vz
            navdataData.ax = localNavdata.ax
            navdataData.ay = localNavdata.ay
            navdataData.az = localNavdata.az

            navdataData.tagsCount = localNavdata.tagsCount
            navdataData.tagsType = localNavdata.tagsType
            navdataData.tagsXc = localNavdata.tagsXc
            navdataData.tagsYc = localNavdata.tagsYc
            navdataData.tagsWidth = localNavdata.tagsWidth
            navdataData.tagsHeight = localNavdata.tagsHeight
            navdataData.tagsOrientation = localNavdata.tagsOrientation
            navdataData.tagsDistance = localNavdata.tagsDistance

            navdataData.timeStamp = localNavdata.tm

            self.lock.acquire()
            self.navData = navdataData
            self.lock.release()

    def hasproxy (self):
        '''
        Returns if proxy has ben created or not. 

        @return if proxy has ben created or not (Boolean)

        '''
        return hasattr(self,"proxy") and self.proxy

    def getNavdata(self):      
        if self.hasproxy():
            self.lock.acquire()
            navData = self.navData
            self.lock.release()
            return navData

        return None




class NavdataIceClient:
    def __init__(self,ic,prefix, start = False):
        self.navdata = NavData(ic,prefix)

        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self.navdata, self.kill_event)
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

    def getNavData(self):
        return self.navdata.getNavdata()

    def hasproxy (self):
        '''
        Returns if proxy has ben created or not. 

        @return if proxy has ben created or not (Boolean)

        '''
        return self.navdata.hasproxy()