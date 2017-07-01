/*
 *  Copyright (C) 1997-2017 JDE Developers Team
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
 *       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>	
 */

#include "evicam_driver/interfaces/pantilti.hpp"

namespace pantilt
{
	PanTiltI::PanTiltI(EVI_D100P* cam)
	{
		std::cout << "PanTilt start" << std::endl;
		this->cam = cam;

	}
	
	PanTiltI::~PanTiltI()
	{
	
	}
	
	Ice::Int PanTiltI::setPTMotorsData(const jderobot::PTMotorsDataPtr& data, const Ice::Current& c)
	{
		int pp = data->pan;
		if (pp< EVILIB_minpan){
			pp = EVILIB_minpan;
		}
		if (pp > EVILIB_maxpan){
			pp = EVILIB_maxpan;
		}
		int tp = data->tilt;
		if (tp< EVILIB_mintilt){
			tp = EVILIB_mintilt;
		}
		if (tp > EVILIB_maxtilt){
			tp = EVILIB_maxtilt;
		}
		
		// use max speed values because cam only accepts this values to move
		this->cam->Pan_TiltDrive(EVILIB_ABSOLUTE, EVILIB_max_pspeed, EVILIB_max_tspeed, pp, tp, EVILIB_NO_WAIT_COMP);

		return 0;
	}

	jderobot::PTMotorsParamsPtr PanTiltI::getPTMotorsParams(Ice::Current const&){
		jderobot::PTMotorsParams * data = new jderobot::PTMotorsParams();
		data->maxPan = EVILIB_maxpan;
		data->minPan = EVILIB_minpan;
		data->maxTilt = EVILIB_maxtilt;
		data->minTilt = EVILIB_mintilt;
		data->maxPanSpeed = EVILIB_max_pspeed;
		data->maxTiltSpeed = EVILIB_max_tspeed;

		jderobot::PTMotorsParamsPtr dataptr (data);

		return dataptr;



	}
}