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
#include "jderobot/comm/ice/bumperIceClient.hpp"

namespace Comm {

BumperIceClient::BumperIceClient(Comm::Communicator* jdrc, std::string prefix) {

	this->prefix=prefix;
	
	this->refreshRate=0;
	float fps=jdrc->getConfig().asFloatWithDefault(prefix+".Fps",10);
	this->cycle=(1/fps)*1000000;


	std::string proxy = jdrc->getConfig().asString(prefix+".Proxy");
	Ice::ObjectPrx baseBumper = jdrc->getIceComm()->stringToProxy(proxy);

	if (0==baseBumper){
		this->on = false;
		std::cout << prefix + ".Proxy configuration not specified" <<std::endl;

	}
	else {

		try{
			this->prx = jderobot::BumperPrx::checkedCast(baseBumper);

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

	this->pauseStatus=false;

}

BumperIceClient::~BumperIceClient() {
	this->on=false;
}


void BumperIceClient::pause(){
	std::cout << "Paused" << std::endl;
	this->pauseStatus=true;
}

void BumperIceClient::resume(){
	std::cout << "resumed" << std::endl;
	this->controlMutex.lock();
		this->pauseStatus=false;
		this->sem.broadcast();
	this->controlMutex.unlock();
}

void BumperIceClient::run(){

	IceUtil::Time last;

	last=IceUtil::Time::now();
	while (this->on){
		if (pauseStatus){
			IceUtil::Mutex::Lock sync(this->controlMutex);
			this->sem.wait(sync);
		}

		try{
			jderobot::BumperDataPtr localBumper=this->prx->getBumperData();

			this->controlMutex.lock();
			this->bumperData.state = localBumper->state;
    		this->bumperData.bumper = localBumper->bumper;
    		this->controlMutex.unlock();

			
		}
		catch(...){
			LOG(WARNING) <<prefix +"error during request (connection error)";
			usleep(5000);

		}


		if ((IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()) > this->cycle ){
			DLOG(WARNING) <<prefix + ": Bumper adquisition timeout-";
		}
		else{
			usleep(this->cycle - (IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()));
		}
		this->refreshRate=(int)(1000000/(IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()));
		last=IceUtil::Time::now();
	}
}

JdeRobotTypes::BumperData  BumperIceClient::getBumperData(){
	JdeRobotTypes::BumperData bumper;
	this->controlMutex.lock();
	bumper = this->bumperData;
	this->controlMutex.unlock();
	return bumper;
}

} /* NS */