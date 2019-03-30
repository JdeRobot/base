/*
 *  Copyright (C) 1997-2015 JDE Developers Team
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
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */


#include "quadrotor/interfaces/cmdveli.h"
// #include "quadrotor/control/twist.hh"

using namespace quadrotor::interfaces;
using namespace jderobot;
using namespace ignition::math;

CMDVelI::CMDVelI (QuadrotorControl *control):
    control(control)
{}

CMDVelI::~CMDVelI ()
{}


Ice::Int
CMDVelI::setCMDVelData(const jderobot::CMDVelDataPtr& data, const Ice::Current&){
	Twist twist_cmd;
	twist_cmd.linear = Vector3d(data->linearX, data->linearY, data->linearZ);
	twist_cmd.angular = Vector3d(data->angularX, data->angularY, data->angularZ);

	control->setTargetVelocity(twist_cmd);

	return 0;
}
