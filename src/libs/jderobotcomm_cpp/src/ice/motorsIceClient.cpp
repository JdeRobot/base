/*
 *  Copyright (C) 1997-2017 JDE Developers
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
 *  Author : Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
 */

#include "jderobot/comm/ice/motorsIceClient.hpp"

namespace JdeRobotComm {

MotorsIceClient::MotorsIceClient(Ice::CommunicatorPtr ic, std::string prefix) {

	this->prefix=prefix;
	Ice::PropertiesPtr prop;
	prop = ic->getProperties();


	Ice::ObjectPrx baseMotors = ic->propertyToProxy(prefix+".Proxy");

	if (0==baseMotors){
		this->on = false;
		std::cout << prefix + ".Proxy configuration not specified" <<std::endl;

	}
	else {

		try{
			this->prx = jderobot::MotorsPrx::checkedCast(baseMotors);

			if (0 == this->prx){
				this->on = false;
	 	   		std::cout <<"Invalid proxy "+ prefix + ".Proxy" <<std::endl;
	 		}else{
	 			this->on = true;
	 			std::cout << prefix + " connected" << std::endl;
	 		}

		
		}catch (const Ice::ConnectionRefusedException& e) {
			std::cout << prefix +" inactive" << std::endl;
		}
		catch (const Ice::Exception& ex) {
			std::cerr << ex << std::endl;
		}
	}

}

MotorsIceClient::~MotorsIceClient() {
	this->on=false;
}

void
MotorsIceClient::sendVelocities(JdeRobotTypes::CMDVel vel){
	this->controlMutex.lock();
		this->prx->setV(vel.vx);
		this->prx->setL(vel.vy);
		this->prx->setW(vel.az);
	this->controlMutex.unlock();
}

void
MotorsIceClient::sendVX (float vx){
	this->controlMutex.lock();
		this->prx->setV(vx);
	this->controlMutex.unlock();
}

void
MotorsIceClient::sendVY (float vy){
	this->controlMutex.lock();
		this->prx->setL(vy);
	this->controlMutex.unlock();

}

void
MotorsIceClient::sendAZ (float az){
	this->controlMutex.lock();
		this->prx->setW(az);
	this->controlMutex.unlock();
}

void
MotorsIceClient::sendV (float v){
	this->sendVX(v);
}

void
MotorsIceClient::sendW (float w){
	this->sendAZ(w);
}

void
MotorsIceClient::sendL (float l ){
	this->sendVY(l);
}



} /* NS */