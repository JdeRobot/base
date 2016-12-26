/*
 *  Copyright (C) 1997-2016 JDE Developers Team
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

#ifndef JDEROBOTCOM_LASERCLIENT_H
#define JDEROBOTCOM_LASERCLIENT_H

#include <jderobot/types/laserData.h>
#include <Ice/Communicator.h>
#include <Ice/Properties.h>
#include <jderobot/com/interfaces/laserClient.hpp>
#include <jderobot/com/ice/laserIceClient.hpp>




namespace JdeRobotCom {

	/**
	 * @brief make a LaserClient using propierties
	 *
	 *
	 * @param communicator that contains properties
	 * @param prefix of client Propierties (example: "kobukiViewer.Laser")
	 * 
	 *
	 * @return null if propierties are wrong
	 */
	LaserClient* getLaserClient(Ice::CommunicatorPtr ic, std::string prefix);


} //NS

#endif // JDEROBOTCOM_LASERCLIENT_H