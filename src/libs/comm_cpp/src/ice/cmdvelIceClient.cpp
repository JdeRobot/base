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

#include "jderobot/comm/ice/cmdvelIceClient.hpp"

namespace Comm {

CMDVelIceClient::CMDVelIceClient(Comm::Communicator* jdrc, std::string prefix) {

	this->prefix=prefix;

	std::string proxy = jdrc->getConfig().asString(prefix+".Proxy");
	Ice::ObjectPrx baseCMDVel = jdrc->getIceComm()->stringToProxy(proxy);

	if (0==baseCMDVel){
		this->on = false;
		std::cout << prefix + ".Proxy configuration not specified" <<std::endl;

	}
	else {

		try{
			this->prx = jderobot::CMDVelPrx::checkedCast(baseCMDVel);

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

CMDVelIceClient::~CMDVelIceClient() {
	this->on=false;
}

void
CMDVelIceClient::sendVelocities(JdeRobotTypes::CMDVel vel){
	this->controlMutex.lock();
		jderobot::CMDVelDataPtr icevel=new jderobot::CMDVelData();

        icevel->linearX=vel.vx;
        icevel->linearY=vel.vy;
        icevel->linearZ=vel.vz;
        icevel->angularZ=vel.az;
        icevel->angularX=vel.ax;
        icevel->angularY=vel.ay;

        this->prx->setCMDVelData(icevel);
	this->controlMutex.unlock();
}


} /* NS */