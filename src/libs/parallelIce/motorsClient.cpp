/*
 *  Copyright (C) 1997-2013 JDE Developers TeamkinectViewer.camRGB
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
 *
 *  Author : Jose María Cañas <jmplaza@gsyc.es>
			Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>

 */
#include "motorsClient.h"

namespace jderobot {

motorsClient::motorsClient(Ice::CommunicatorPtr ic, std::string prefix, bool debug) {
	// TODO Auto-generated constructor stub
	this->prefix=prefix;
	this->debug= debug;
	Ice::PropertiesPtr prop;
	prop = ic->getProperties();

	try{

		std::string maxWstr = prop->getPropertyWithDefault(prefix+".maxW","0.5");
        this->maxW = atof(maxWstr.c_str());
  
        std::string maxVstr = prop->getPropertyWithDefault(prefix+".maxV","0.5");
        this->maxV = atof(maxVstr.c_str());

        if (debug){
			std::cout << "maxV value:" << this->maxV << std::endl;
			std::cout << "maxW value:" << this->maxW << std::endl;
        }
        
		Ice::ObjectPrx baseMotors = ic->propertyToProxy(prefix+"Proxy");
		this->prx = jderobot::MotorsPrx::checkedCast(baseMotors);
		if (0==this->prx) {
			std::cerr << "Interface " << prefix << " not configured "<< std::endl;;
		}

	}catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	}
	catch (const char* msg) {
		std::cerr << msg << std::endl;
		jderobot::Logger::getInstance()->error(prefix + " Not motors provided");
	}
}

motorsClient::~motorsClient() {
	// TODO Auto-generated destructor stub
}

void motorsClient::setV(float v) {
	this->controlMutex.lock();
	this->v = v;
	this->controlMutex.unlock();

}

void motorsClient::setW(float w) {
	this->controlMutex.lock();
	this->w = w;
	this->controlMutex.unlock();

}

float motorsClient::getMaxV() {
	return this->maxV;
}

float motorsClient::getMaxW() {
	return this->maxW;
}

void motorsClient::sendVelocities() {
	if (this->hasProxy()){
		this->controlMutex.lock();
		float v = this->v;
		float w = this->w;
		this->controlMutex.unlock();
		this->prx->setV(v);
		this->prx->setW(w);
	}
}

void motorsClient::sendV(float v) {
	if (this->hasProxy()){
		this->controlMutex.lock();
		this->prx->setV(v);
		this->controlMutex.unlock();
	}

}

void motorsClient::sendW(float w) {
	if (this->hasProxy()){
		this->controlMutex.lock();
		this->prx->setW(w);
		this->controlMutex.unlock();
	}
}

bool motorsClient::hasProxy(){
	return this->prx;
}

} /* namespace eldercare */
