# -*- coding: utf-8 -*-
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


class NavdataData ():

	def __init__(self):

		self.vehicle = 1 # 0-> ArDrone1, 1-> ArDrone2
		self.state = 0 
		self.batteryPercent = 0 # The remaing charge of baterry %
		
		#Magnetometer ArDrone 2.0
		self.magX = 0 ;
		self.magY = 0 ;
		self.magZ = 0 ;

		
		self.pressure = 0 ; #Barometer ArDrone 2.0
		self.temp = 0 ; #Temperature sensor ArDrone 2.0

		self.windSpeed= 0 ;
		self.windAngle= 0 ;
		self.windCompAngle= 0 ;
		
		self.rotX= 0 ; #rotation about the X axis
		self.rotY= 0 ; #rotation about the Y axis
		self.rotZ= 0 ; #rotation about the Z axis       
		self.altd= 0 ; #Estimated altitude (mm)  
		self.vx= 0 ; #linear velocity (mm/sec)
		self.vy= 0 ; #linear velocity (mm/sec)
		self.vz= 0 ; #linear velocity (mm/sec)
		#linear accelerations (unit: g) ¿ArDrone 2.0?
		self.ax= 0 ;
		self.ay= 0 ;
		self.az= 0 ;
		
		#Tags in Vision Detectoion
		#Should be unsigned
		self.tagsCount= 0 ;

		self.tagsType = [];
		self.tagsXc = [];
		self.tagsYc = [];
		self.tagsWidth = [];
		self.tagsHeight = [];
		self.tagsOrientation = [];
		self.tagsDistance = [];

		self.timeStamp = 0 # seconds




	def __str__(self):
		s = "NavdataData: {\n   vehicle: " + str(self.vehicle) + "\n   state: " + str(self.state)
		s = s + "\n   batteryPercent: " + str(self.batteryPercent) + "\n   rotX: " + str(self.rotX) 
		s = s + "\n   rotY: " + str(self.rotY) + "\n   rotZ: " + str(self.rotZ) 
		s = s + "\n   altd: " + str(self.altd) + "\n   vx: " + str(self.vx) 
		s = s + "\n   vy: " + str(self.vy) + "\n   vz: " + str(self.vz) 
		s = s + "\n   timeStamp: " + str(self.timeStamp) + "\n...}"
		return s 