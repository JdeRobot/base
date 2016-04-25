/*
 * ProximitySensor.cpp
 *
 *  Created on: 7 de abr. de 2016
 *      Author: roberto
 */

#include <boost/thread/lock_guard.hpp> 
#include "proximitySensorI.h"

namespace EMSensor {

ProximitySensorI::ProximitySensorI() {
	this->distance = 0;
	this->time_stamp = 0;
	this->status = jderobot::State::Error;

}

void ProximitySensorI::setData(float t, float d, jderobot::State status){
	boost::lock_guard<boost::mutex> lock(this->synch);
	this->distance = d;
	this->time_stamp = t;
	this->status = status;

}

jderobot::ProximitySensorDataPtr ProximitySensorI::getProximitySensorData(const ::Ice::Current&){

	jderobot::ProximitySensorDataPtr prxSensorData (new jderobot::ProximitySensorData());
	boost::lock_guard<boost::mutex> lock(this->synch);

	prxSensorData->d = this->distance;
	prxSensorData->tm = this->time_stamp;
	prxSensorData->status = this->status;

	return prxSensorData;
}



} /* namespace proximitySensor */
