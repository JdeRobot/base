# -*- coding: utf-8 -*-
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

import time

import traceback
import jderobot
import threading
import Ice
from .threadSensor import ThreadSensor


class PTMotors:

    def __init__(self, jdrc, prefix):
        self.lock = threading.Lock()

        self.data=jderobot.PTMotorsData()
        
        self.params=jderobot.PTMotorsParams()
        ic = jdrc.getIc()

        try:
            proxyStr = jdrc.getConfig().getProperty(prefix+".Proxy")
            base = ic.stringToProxy(proxyStr)
            self.proxy = jderobot.PTMotorsPrx.checkedCast(base)

            if not self.proxy:
                print ('Interface ' + prefix + ' not configured')

            else:
                self.params = self.proxy.getPTMotorsParams()

                print ("+++ MAX/MIN Pan/Tilt Values +++")
                print ("+    Min Pan: " + str(self.params.minPan) + "         +")
                print ("+    Max Pan: " + str(self.params.maxPan) + "          +")
                print ("+    Max Pan speed: " + str(self.params.maxPanSpeed) + "     +")
                print ("+    Min Tilt: " + str(self.params.minTilt) + "         +")
                print ("+    Max Tilt: " + str(self.params.maxTilt) + "          +")
                print ("+    Max Tilt speed: " + str(self.params.maxTiltSpeed) + "    +")
                print ("++++++++++++++++++++++++++++++")

        except Ice.ConnectionRefusedException:
            print(prefix + ': connection refused')

        except:
            traceback.print_exc()
            exit(-1)

    def getLimits(self):
        self.lock.acquire()
        params = self.params        
        self.lock.release()

        return params

    def setPTMotorsData(self, pan, tilt, panspeed, tiltspeed):

        self.lock.acquire()
        self.data.pan = pan
        self.data.tilt = tilt
        self.data.panSpeed = panspeed
        self.data.tiltSpeed = tiltspeed
        self.lock.release()

        

    def sendPTMotorsData(self):
        if self.hasproxy():
            self.lock.acquire()
            self.data.timeStamp = time.time()
            self.proxy.setPTMotorsData(self.data)
            self.lock.release()

    def hasproxy (self):
        return hasattr(self,"proxy") and self.proxy

    def update(self):
        self.sendPTMotorsData()



class PTMotorsIceClient:
    def __init__(self,ic,prefix, start = False):
        self.motors = PTMotors(ic,prefix)

        self.kill_event = threading.Event()
        self.thread = ThreadSensor(self.motors, self.kill_event)
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

    def getLimits(self):
        return self.motors.getLimits()

    def hasproxy():
        return self.motors.hasproxy()

    def setPTMotorsData(self, pan, tilt, panspeed, tiltspeed):
        self.motors.setPTMotorsData(pan, tilt, panspeed, tiltspeed)
