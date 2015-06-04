#
#  Copyright (C) 1997-2015 JDE Developers Team
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
#
import sys, traceback, Ice
import jderobot
import numpy as np
import threading
from sensors.colorFilterValues import ColorFilterValues

class Sensor:
    ARDRONE1=0
    ARDRONE2=1
    ARDRONE_SIMULATED=10

    def __init__(self):
        self.lock = threading.Lock()

        self.playButton=False

        self.cmd=jderobot.CMDVelData()
        self.cmd.linearX=self.cmd.linearY=self.cmd.linearZ=0
        self.cmd.angularZ=0
        ''' With values distinct to 0 in the next fields, the ardrone not enter in hover mode'''
        self.cmd.angularX=0.5
        self.cmd.angularY=1.0


        try:
            ic = Ice.initialize(sys.argv)
            properties = ic.getProperties()
            basecamera = ic.propertyToProxy("Introrob.Camera.Proxy")
            self.cameraProxy = jderobot.CameraPrx.checkedCast(basecamera)

            if self.cameraProxy:
                self.image = self.cameraProxy.getImageData("RGB8")
                self.height= self.image.description.height
                self.width = self.image.description.width

                self.trackImage = np.zeros((self.height, self.width,3), np.uint8)
                self.trackImage.shape = self.height, self.width, 3

                self.thresoldImage = np.zeros((self.height, self.width,1), np.uint8)
                self.thresoldImage.shape = self.height, self.width, 1

                self.filterValues = ColorFilterValues()

            else:
                print 'Interface camera not connected'

            basecmdVel = ic.propertyToProxy("Introrob.CMDVel.Proxy")
            self.cmdVelProxy=jderobot.CMDVelPrx.checkedCast(basecmdVel)
            if not self.cmdVelProxy:
                print 'Interface cmdVel not connected'

            basenavdata = ic.propertyToProxy("Introrob.Navdata.Proxy")
            self.navdataProxy = jderobot.NavdataPrx.checkedCast(basenavdata)

            if self.navdataProxy:
                self.navdata=self.navdataProxy.getNavdata()
                if self.navdata.vehicle == self.ARDRONE_SIMULATED :
                    self.virtualDrone = True
                else:
                    self.virtualDrone = False
            else:
                print 'Interface navdata not connected'
                self.virtualDrone = True

            baseextra = ic.propertyToProxy("Introrob.Extra.Proxy")
            self.extraProxy=jderobot.ArDroneExtraPrx.checkedCast(baseextra)
            if not self.extraProxy:
                print 'Interface ardroneExtra not connected'

            basepose3D = ic.propertyToProxy("Introrob.Pose3D.Proxy")
            self.pose3DProxy=jderobot.Pose3DPrx.checkedCast(basepose3D)
            if self.pose3DProxy:
                self.pose=jderobot.Pose3DData()
            else:
                print 'Interface pose3D not connected'

        except:
            traceback.print_exc()
	    exit()
            status = 1
            
    def update(self):
        self.lock.acquire()
        self.updateCamera()
        self.updateNavdata()
        self.updatePose()
        self.lock.release()

    def updateCamera(self):
        if self.cameraProxy:
            self.image = self.cameraProxy.getImageData("RGB8")
            self.height= self.image.description.height
            self.width = self.image.description.width

    def updateNavdata(self):
        if self.navdataProxy:
            self.navdata=self.navdataProxy.getNavdata()

    def updatePose(self):
        if self.pose3DProxy:
            self.pose=self.pose3DProxy.getPose3DData()

    def getNavdata(self):
        if self.navdataProxy:
            self.lock.acquire()
            tmp=self.navdata
            self.lock.release()
            return tmp

        return None
    
    def getImage(self):
        if self.cameraProxy:
            self.lock.acquire()
            img = np.zeros((self.height, self.width, 3), np.uint8)
            img = np.frombuffer(self.image.pixelData, dtype=np.uint8)
            img.shape = self.height, self.width, 3
            self.lock.release()
            return img;

        return None

    def getTrackImage(self):
        if self.cameraProxy:
            self.lock.acquire()
            img = np.zeros((self.height, self.width, 3), np.uint8)
            img = self.trackImage
            img.shape = self.trackImage.shape
            self.lock.release()
            return img;
        return None

    def setTrackImage(self, image):
        if self.cameraProxy:
            self.lock.acquire()
            self.trackImage = image
            self.trackImage.shape = image.shape
            self.lock.release()

    def getThresoldImage(self):
        if self.cameraProxy:
            self.lock.acquire()
            img = np.zeros((self.height, self.width, 1), np.uint8)
            img = self.thresoldImage
            img.shape = self.thresoldImage.shape
            self.lock.release()
            return img;
        return None

    def setThresoldImage(self, image):
        if self.cameraProxy:
            self.lock.acquire()
            self.thresoldImage = image
            self.thresoldImage.shape = image.shape
            self.lock.release()

    def setColorFilterValues(self, values):
        self.filterValues = values

    def getColorFilterValues(self):
        return self.filterValues

    def takeoff(self):
        if self.extraProxy:
            self.lock.acquire()
            self.extraProxy.takeoff()
            self.lock.release()
        
    def land(self):
        if self.extraProxy:
            self.lock.acquire()
            self.extraProxy.land()
            self.lock.release()
        
    def toggleCam(self):
        if self.extraProxy:
            self.lock.acquire()
            self.extraProxy.toggleCam()
            self.lock.release()
              
    def reset(self):
        if self.extraProxy:
            self.lock.acquire()
            self.extraProxy.reset()
            self.lock.release()
        
    def record(self,record):
        if self.extraProxy and self.navdataProxy:
            if self.navdata.vehicle == self.ARDRONE2:
                self.extraProxy.recordOnUsb(record)

    def setVX(self,vx):
        self.cmd.linearX=vx

    def setVY(self,vy):
        self.cmd.linearY=vy
        
    def setVZ(self,vz):
        self.cmd.linearZ=vz

    def setYaw(self,yaw):
        self.cmd.angularZ=yaw        
        
    def setRoll(self,roll):
        self.cmd.angularX=roll 
        
    def setPitch(self,pitch):
        self.cmd.angularY=pitch                 
                
    def Velocities(self):
        if self.cmdVelProxy:
            self.lock.acquire();
            self.cmdVelProxy.setCMDVelData(self.cmd)
            self.lock.release();

    def sendVelocities(self):
        if self.cmdVelProxy:
            self.lock.acquire();
            self.cmdVelProxy.setCMDVelData(self.cmd)
            self.lock.release();

    def CMDVel(self,vx,vy,vz,yaw,roll,pitch):
        cmd=jderobot.CMDVelData()
        cmd.linearX=vy
        cmd.linearY=vx
        cmd.linearZ=vz
        cmd.angularZ=yaw
        cmd.angularX=cmd.angularY=1.0

        if self.cmdVelProxy:
            self.lock.acquire();
            self.cmdVelProxy.setCMDVelData(cmd)
            self.lock.release();
    
    def getPose3D(self):
        if self.pose3DProxy:
            self.lock.acquire()
            tmp=self.pose
            self.lock.release()
            return tmp

        return None
    
    def isPlayButton(self):
        return self.playButton
    
    def setPlayButton(self,value):
        self.playButton=value

    def isVirtual(self):
        return self.virtualDrone
