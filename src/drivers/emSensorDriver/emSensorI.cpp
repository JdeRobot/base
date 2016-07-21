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
 *       Luis Roberto Morales Iglesias <lr.morales.iglesias@gmail.com>	
 */

#include <boost/thread/lock_guard.hpp> 
#include "emSensorI.h"

namespace EMSensor {

EMSensorI::EMSensorI() {
	this->distance = 0;
	this->time_stamp = 0;
	this->status = jderobot::State::Error;

}

void EMSensorI::setData(float t, float d, jderobot::State status){
	boost::lock_guard<boost::mutex> lock(this->synch);
	this->distance = d;
	this->time_stamp = t;
	this->status = status;

}

jderobot::EMSensorDataPtr EMSensorI::getEMSensorData(const ::Ice::Current&){

	jderobot::EMSensorDataPtr emSensorData (new jderobot::EMSensorData());
	boost::lock_guard<boost::mutex> lock(this->synch);

	emSensorData->d = this->distance;
	emSensorData->tm = this->time_stamp;
	emSensorData->status = this->status;

	return emSensorData;
}



} /* namespace EMSensor */
