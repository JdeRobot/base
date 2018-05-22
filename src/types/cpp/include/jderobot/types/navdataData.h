/*
 *  Copyright (C) 1997-2016 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY= 0 ; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *  Authors :
 *       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
 */

#ifndef JDEROBOTTYPES_NAVDATADATA_H
#define JDEROBOTTYPES_NAVDATADATA_H

#include <vector>

namespace JdeRobotTypes {

	class NavdataData {
	public:
	    //0-> ArDrone1, 1-> ArDrone2
		int vehicle = 1;
		int state = 0;
		//The remaing charge of baterry %
		float batteryPercent = 0;
		//Magnetometer ArDrone 2.0
		int magX= 0 ;
		int magY= 0 ;
		int magZ= 0 ;
		//Barometer ArDrone 2.0
		int pressure= 0 ;
		//Temperature sensor ArDrone 2.0
		int temp= 0 ;
		//Estimated wind speed ArDrone 2.0
		float windSpeed= 0 ;
		float windAngle= 0 ;
		float windCompAngle= 0 ;
		//rotation about the X axis
		float rotX= 0 ;
		//rotation about the Y axis
		float rotY= 0 ;
		//rotation about the Z axis
		float rotZ= 0 ;  
		//Estimated altitude (mm)              
		int altd= 0 ;
		//linear velocity (mm/sec)
		float vx= 0 ;
		//linear velocity (mm/sec)
		float vy= 0 ;
		//linear velocity (mm/sec)
		float vz= 0 ;
		//linear accelerations (unit: g) ¿ArDrone 2.0?
		float ax= 0 ;
		float ay= 0 ;
		float az= 0 ;
		//Tags in Vision Detectoion
		//Should be unsigned
		int tagsCount= 0 ;

		std::vector<int> tagsType;
		std::vector<int> tagsXc;
		std::vector<int> tagsYc;
		std::vector<int> tagsWidth;
		std::vector<int> tagsHeight;
		std::vector<float> tagsOrientation;
		std::vector<float> tagsDistance;
	    double timeStamp = 0; //seconds
	};


} //NS

#endif // JDEROBOTTYPES_NAVDATADATA_H


