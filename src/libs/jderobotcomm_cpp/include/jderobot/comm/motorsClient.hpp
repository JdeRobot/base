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

#ifndef JDEROBOTCOMM_MOTORSCLIENT_H
#define JDEROBOTCOMM_MOTORSCLIENT_H

#include <jderobot/types/cmdvel.h>
#include <Ice/Communicator.h>
#include <Ice/Properties.h>
#include <jderobot/comm/interfaces/motorsClient.hpp>
#include <jderobot/comm/ice/motorsIceClient.hpp>
#include <jderobot/comm/ros/publisherMotors.hpp>




namespace JdeRobotComm {

	/**
	 * @brief make a MotorsClient using propierties
	 *
	 *
	 * @param communicator that contains properties
	 * @param prefix of client Propierties (example: "kobukiViewer.Motors")
	 * 
	 *
	 * @return null if propierties are wrong
	 */
	MotorsClient* getMotorsClient(Ice::CommunicatorPtr ic, std::string prefix);


} //NS

#endif // JDEROBOTCOMM_MOTORSCLIENT_H