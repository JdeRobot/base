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

#include "jderobot/comm/ice/ardroneextraIceClient.hpp"

namespace Comm {

ArDroneExtraIceClient::ArDroneExtraIceClient(Comm::Communicator* jdrc, std::string prefix) {

	this->prefix=prefix;

	std::string proxy = jdrc->getConfig().asString(prefix+".Proxy");
	Ice::ObjectPrx baseArDroneExtra = jdrc->getIceComm()->stringToProxy(proxy);

	if (0==baseArDroneExtra){
		this->on = false;
		std::cout << prefix + ".Proxy configuration not specified" <<std::endl;

	}
	else {

		try{
			this->prx = jderobot::ArDroneExtraPrx::checkedCast(baseArDroneExtra);

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

ArDroneExtraIceClient::~ArDroneExtraIceClient() {
	this->on=false;
}

void 
ArDroneExtraIceClient::toggleCam(){
	this->controlMutex.lock();
		this->prx->toggleCam();
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::land(){
	this->controlMutex.lock();
		this->prx->land();
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::takeoff(){
	this->controlMutex.lock();
		this->prx->takeoff();
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::reset(){
	this->controlMutex.lock();
		this->prx->reset();
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::recordOnUsb(bool record){
	this->controlMutex.lock();
		this->prx->recordOnUsb(record);
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::ledAnimation(int type,float duration, float req){
	this->controlMutex.lock();
		this->prx->ledAnimation(type, duration, req);
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::flightAnimation(int type, float duration){
	this->controlMutex.lock();
		this->prx->flightAnimation(type, duration);
	this->controlMutex.unlock();
}
void
ArDroneExtraIceClient::flatTrim(){
	this->controlMutex.lock();
		this->prx->flatTrim();
	this->controlMutex.unlock();
}


} /* NS */