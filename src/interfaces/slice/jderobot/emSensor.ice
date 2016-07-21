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
 *       Luis Roberto Morales Iglesias <lr.morales.iglesias@gmail.com>
 */

#ifndef EMSENSOR_ICE
#define EMSENSOR_ICE

module jderobot{

	enum State { Error, OutRange, FarRange, NearRange, Close }; 
	
	class EMSensorData 
	{
	        
		State status;
		float d;
		//time stamp
		long tm;
	};
	
	interface EMSensor
	{
		idempotent EMSensorData getEMSensorData();
	};	
};

#endif
