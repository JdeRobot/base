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
import threading
import Ice
from .threadSensor import ThreadSensor
from jderobotTypes import CMDVel


class CMDVelIceClient:


    '''
        CMDVel Contructor.
        Exits When it receives a Exception diferent to Ice.ConnectionRefusedException

        @param jdrc: Comm Communicator
        @param prefix: prefix name of client in config file

        @type ic: Ice Communicator
        @type prefix: String
        '''
    def __init__(self, jdrc, prefix):
        self.lock = threading.Lock()
        ic = jdrc.getIc()

        self.vel = CMDVel()
        

        try:
            proxyStr = jdrc.getConfig().getProperty(prefix+".Proxy")
            base = ic.stringToProxy(proxyStr)
            self.proxy = jderobot.CMDVelPrx.checkedCast(base)


            if not self.proxy:
                print ('Interface ' + prefix + ' not configured')

        except Ice.ConnectionRefusedException:
            print(prefix + ': connection refused')

        except:
            traceback.print_exc()
            exit(-1)


    def start(self):
        pass
    def stop(self):
        pass
        
    def setVX(self,vx):
        self.lock.acquire()
        self.vel.vx=vx
        self.lock.release()

    def setVY(self,vy):
        self.lock.acquire()
        self.vel.vy=vy
        self.lock.release()

    def setVZ(self,vz):
        self.lock.acquire()
        self.vel.vz=vz
        self.lock.release()

    def setAngularZ(self,az):
        self.lock.acquire()
        self.vel.az=az
        self.lock.release()

    def setAngularX(self,ax):
        self.lock.acquire()
        self.vel.ax=ax
        self.lock.release()
        
    def setAngularY(self,ay):
        self.lock.acquire()
        self.vel.ay=ay
        self.lock.release()

    def setYaw(self,yaw):
       self.setAngularZ(yaw)

    def setRoll(self,roll):
        self.angularX(roll)
        
    def setPitch(self,pitch):
        self.angularY(pitch)

    def setCMD (self, cmd):
        self.lock.acquire()
        self.vel = cmd
        self.lock.release()

    def sendVelocities(self):
        if self.hasproxy():
            self.lock.acquire()
            tmp = self.vel
            self.lock.release()
            self.sendCMDVel(tmp.vx,tmp.vy,tmp.vz,tmp.ax, tmp.ay, tmp.az)

    def sendCMDVel(self,vx,vy,vz,ax,ay,az):
        cmd=jderobot.CMDVelData()

        cmd.linearX = vx
        cmd.linearY = vy
        cmd.linearZ = vz
        cmd.angularZ = az
        cmd.angularX = ax
        cmd.angularY = ay

        if self.hasproxy():
            self.lock.acquire()
            self.proxy.setCMDVelData(cmd)
            self.lock.release()


    def hasproxy (self):
        return hasattr(self,"proxy") and self.proxy