/*
 *  Copyright (C) 1997-2014 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *  Authors : 
 *       Alberto Martín Florido <almartinflorido@gmail.com>	
 */

#ifndef NAVDATA_ICE
#define NAVDATA_ICE

module jderobot{
	sequence<int> arrayInt;
	sequence<float> arrayFloat;
	
	class NavdataData 
	{
		//0-> ArDrone1, 1-> ArDrone2
		int vehicle;
		int state;
		//The remaing charge of baterry %
		float batteryPercent;
		//Magnetometer ArDrone 2.0
		int magX;
		int magY;
		int magZ;
		//Barometer ArDrone 2.0
		int pressure;
		//Temperature sensor ArDrone 2.0
		int temp;
		//Estimated wind speed ArDrone 2.0
		float windSpeed;
		float windAngle;
		float windCompAngle;
		//rotation about the X axis
		float rotX;
		//rotation about the Y axis
		float rotY;
		//rotation about the Z axis
		float rotZ;  
		//Estimated altitude (mm)              
		int altd;
		//linear velocity (mm/sec)
		float vx;
		//linear velocity (mm/sec)
		float vy;
		//linear velocity (mm/sec)
		float vz;
		//linear accelerations (unit: g) ¿ArDrone 2.0?
		float ax;
		float ay;
		float az;
		//Tags in Vision Detectoion
		//Should be unsigned
		int tagsCount;
		arrayInt tagsType;
		arrayInt tagsXc;
		arrayInt tagsYc;
		arrayInt tagsWidth;
		arrayInt tagsHeight;
		arrayFloat tagsOrientation;
		arrayFloat tagsDistance;
		//time stamp
		float tm;
	};
	
	interface Navdata
	{
		idempotent NavdataData getNavdata();
	};	
};

#endif
